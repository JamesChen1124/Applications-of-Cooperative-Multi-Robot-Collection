//Final version

#include <Arduino.h>
#include <Servo.h>

// ────────────────────────────────────────
//  Robotic Arm Servo Definitions
// ────────────────────────────────────────
Servo motorA;   // SG90  - Base
Servo motorB;   // LD-220MG - Shoulder
Servo motorC;   // LD-220MG - Elbow
Servo motorD;   // MG996R - Gripper

int posA = 90,  posB = 90,  posC = 90,  posD = 0;            // current angles
int targetA = 90, targetB = 90, targetC = 90, targetD = 0;   // target angles

const int everyA = 25;   // ms / degree (smooth stepping speed)
const int everyB = 15;
const int everyC = 15;
const int everyD = 15;

bool movingA = false, movingB = false, movingC = false, movingD = false;
int  stepNow  = -1;      // -1: idle, 0-12: sequence step index
bool armActive = false;  // flag indicating the arm sequence is running

// ────────────────────────────────────────
//  Chassis Motor Pins (unchanged)
// ────────────────────────────────────────
const int R1   = 22; // right-rear IN1
const int R2   = 23; // right-rear IN2
const int R3   = 24; // right-front IN3
const int R4   = 25; // right-front IN4
const int ENA1 = 5;  // right-rear PWM
const int ENB1 = 4;  // right-front PWM

const int L1   = 27; // left-front IN1
const int L2   = 26; // left-front IN2
const int L3   = 28; // left-rear IN3
const int L4   = 29; // left-rear IN4
const int ENA2 = 6;  // left-front PWM
const int ENB2 = 7;  // left-rear PWM

// ────────────────────────────────────────
/*  Communication & Timeout */
// ────────────────────────────────────────
const unsigned long RX_TIMEOUT = 100;  // ms
unsigned long lastRx = 0;
String incoming;

// ────────────────────────────────────────
//  PWM Range
// ────────────────────────────────────────
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ────────────────────────────────────────
//  Chassis Control Helpers
// ────────────────────────────────────────
void stopAll() {
  analogWrite(ENA1, 0); analogWrite(ENB1, 0);
  analogWrite(ENA2, 0); analogWrite(ENB2, 0);
  digitalWrite(R1, LOW); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW); digitalWrite(R4, LOW);
  digitalWrite(L1, LOW); digitalWrite(L2, LOW);
  digitalWrite(L3, LOW); digitalWrite(L4, LOW);
}

void driveForward(int pwm) {
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
}

void driveBackward(int pwm) {
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
}

void turnLeft(int pwm) {
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
}

void turnRight(int pwm) {
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
}

// ────────────────────────────────────────
//  Smooth Servo Motion for the Arm
// ────────────────────────────────────────
void checkServoA() {
  static uint32_t t = 0; if (millis() - t < everyA) return; t = millis();
  if      (posA < targetA) posA++;
  else if (posA > targetA) posA--;
  else { movingA = false; return; }
  motorA.write(posA);
}
void checkServoB() {
  static uint32_t t = 0; if (millis() - t < everyB) return; t = millis();
  if      (posB < targetB) posB++;
  else if (posB > targetB) posB--;
  else { movingB = false; return; }
  motorB.write(posB);
}
void checkServoC() {
  static uint32_t t = 0; if (millis() - t < everyC) return; t = millis();
  if      (posC < targetC) posC++;
  else if (posC > targetC) posC--;
  else { movingC = false; return; }
  motorC.write(posC);
}
void checkServoD() {
  static uint32_t t = 0; if (millis() - t < everyD) return; t = millis();
  if      (posD < targetD) posD++;
  else if (posD > targetD) posD--;
  else { movingD = false; return; }
  motorD.write(posD);
}

// ────────────────────────────────────────
//  Arm Grasping Sequence State Machine (12 steps)
// ────────────────────────────────────────
void executeSequence() {
  static unsigned long lastTime = 0;
  static bool waiting = false;

  switch (stepNow) {
    case 0:
      targetC = 180; movingC = true; stepNow++; break;
    case 1:
      if (!movingC && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) { waiting = false; stepNow++; }
      break;
    case 2:
      targetB = 108; movingB = true; stepNow++; break;
    case 3:
      if (!movingB && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) { waiting = false; stepNow++; }
      break;
    case 4:
      targetA = 60; movingA = true; stepNow++; break;
    case 5:
      if (!movingA && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetC = 90;  movingC = true;
        targetB = 180; movingB = true;
        stepNow++;
      }
      break;
    case 6:
      if (!movingC && !movingB && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetD = 180; movingD = true;
        stepNow++;
      }
      break;
    case 7:
      if (!movingD && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetC = 70; movingC = true;
        stepNow++;
      }
      break;
    case 8:
      if (!movingC && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetB = 180; movingB = true;
        stepNow++;
      }
      break;
    case 9:
      if (!movingB && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetA = 90; movingA = true;
        stepNow++;
      }
      break;
    case 10:
      if (!movingA && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetB = 90; movingB = true;
        targetC = 90; movingC = true;
        stepNow++;
      }
      break;
    case 11:
      if (!movingB && !movingC && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        targetD = 0; movingD = true;
        stepNow++;
      }
      break;
    case 12:
      if (!movingD && !waiting) { lastTime = millis(); waiting = true; }
      if (waiting && millis() - lastTime >= 1000) {
        waiting = false;
        Serial.println("✅ Arm sequence DONE");
        stepNow   = -1;
        armActive = false;
      }
      break;
  }
}

// ────────────────────────────────────────
//  Initialization
// ────────────────────────────────────────
void setup() {
  Serial.begin(57600);
  lastRx = millis();

  // Chassis GPIO
  pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT); pinMode(R4, OUTPUT);
  pinMode(ENA1, OUTPUT); pinMode(ENB1, OUTPUT);
  pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT); pinMode(L4, OUTPUT);
  pinMode(ENA2, OUTPUT); pinMode(ENB2, OUTPUT);
  stopAll();

  // Arm servos
  motorA.attach(8, 544, 2400); // (pin, min, max)
  motorB.attach(9);
  motorC.attach(10);
  motorD.attach(11);
  motorA.write(posA);
  motorB.write(posB);
  motorC.write(posC);
  motorD.write(posD);

  Serial.println("Robot2 with Arm ready");
}

// ────────────────────────────────────────
//  Main Loop
// ────────────────────────────────────────
void loop() {
  // 1) Handle serial commands
  while (Serial.available()) {
    incoming = Serial.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() == 0) continue;
    lastRx = millis();

    // ---- S / G are commands without PWM value ----
    if (incoming == "S") {
      stopAll();
      Serial.println("DONE");
      continue;
    }
    if (incoming == "G") {
      if (!armActive) {
        armActive = true;
        stepNow   = 0;
        movingA = movingB = movingC = movingD = false;
        targetA  = posA; targetB = posB;
        targetC  = posC; targetD = posD;
        Serial.println("Arm sequence START");
      }
      Serial.println("DONE");
      continue;
    }

    // ---- Other commands with PWM value ----
    int comma = incoming.indexOf(',');
    String cmd = comma > 0 ? incoming.substring(0, comma) : incoming;
    int pwm    = comma > 0 ? incoming.substring(comma + 1).toInt() : 0;

    if      (cmd == "F")  driveForward(pwm);
    else if (cmd == "B")  driveBackward(pwm);
    else if (cmd == "TL") turnLeft(pwm);
    else if (cmd == "TR") turnRight(pwm);
    else                  stopAll();

    Serial.println("DONE");
  }

  // 2) Stop if communication timeout
  if (millis() - lastRx > RX_TIMEOUT) stopAll();

  // 3) Run arm sequence
  if (armActive) {
    checkServoA();
    checkServoB();
    checkServoC();
    checkServoD();
    executeSequence();
  }
}
