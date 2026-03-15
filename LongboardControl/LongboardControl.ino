#include <Servo.h>

// --- Pin Definitions ---
#define SPEED_SENSOR_PIN 2    // TCRT5000 digital output
#define SERVO1_PIN       10   // Servo 1 - binary trigger
#define SERVO2_PIN       9    // Servo 2 - damper stiffness

// --- Wheel Configuration ---
const float WHEEL_DIAMETER_MM     = 75.0;
const int   PULSES_PER_REVOLUTION = 1;

// --- Configurable via Serial (web app) ---
float triggerSpeed   = 30.0;  // km/h to activate servo 1
float maxSpeedKMH    = 50.0;  // km/h where damper is at max
int   maxDamperAngle = 180;   // max angle for servo 2
float testSpeed      = -1.0;  // -1 = use real sensor

// --- Fixed servo angles ---
const int SERVO1_INACTIVE = 0;
const int SERVO1_ACTIVE   = 90;
const int SERVO2_MIN      = 0;

// --- Speed Calculation ---
const unsigned long CALC_INTERVAL_MS = 500;

volatile unsigned long pulseCount = 0;
unsigned long lastCalcTime        = 0;
float speedKMH                    = 0.0;

Servo servo1;
Servo servo2;

void countPulse() {
  pulseCount++;
}

// Parse commands from web app: T:30, X:50, A:180, M:35, L:
void parseSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 2) return;

  char key = cmd.charAt(0);
  float val = cmd.substring(2).toFloat();

  switch (key) {
    case 'T': triggerSpeed   = val; break; // threshold speed
    case 'X': maxSpeedKMH    = val; break; // max speed
    case 'A': maxDamperAngle = (int)val; break; // max damper angle
    case 'M': testSpeed      = val; break; // test mode speed
    case 'L': testSpeed      = -1.0; break; // live sensor mode
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(SPEED_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), countPulse, RISING);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  servo1.write(SERVO1_INACTIVE);
  servo2.write(SERVO2_MIN);

  Serial.println("READY");
}

void loop() {
  parseSerial();

  unsigned long now = millis();

  if (now - lastCalcTime >= CALC_INTERVAL_MS) {
    detachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN));
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), countPulse, RISING);

    float elapsedSeconds = (now - lastCalcTime) / 1000.0;
    lastCalcTime = now;

    if (testSpeed >= 0) {
      speedKMH = testSpeed;
    } else {
      float circumferenceM = PI * (WHEEL_DIAMETER_MM / 1000.0);
      float revolutions    = (float)pulses / PULSES_PER_REVOLUTION;
      speedKMH             = (revolutions * circumferenceM / elapsedSeconds) * 3.6;
    }

    // Servo 1: binary trigger
    servo1.write(speedKMH >= triggerSpeed ? SERVO1_ACTIVE : SERVO1_INACTIVE);

    // Servo 2: proportional damper (only active above trigger speed)
    int damperAngle = SERVO2_MIN;
    if (speedKMH >= triggerSpeed) {
      long clamped = constrain((long)speedKMH, (long)triggerSpeed, (long)maxSpeedKMH);
      damperAngle  = (int)map(clamped, (long)triggerSpeed, (long)maxSpeedKMH, SERVO2_MIN, maxDamperAngle);
    }
    servo2.write(damperAngle);

    // Output for web app: S:35.0,V1:1,D:90
    Serial.print("S:");
    Serial.print(speedKMH, 1);
    Serial.print(",V1:");
    Serial.print(speedKMH >= triggerSpeed ? 1 : 0);
    Serial.print(",D:");
    Serial.println(damperAngle);
  }
}
