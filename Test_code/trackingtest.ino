// -----------------------------
// Arduino: Vision + Ultrasonic + Motor
// -----------------------------

// --- 馬達腳位---
const int R1   = 22; // Right rear wheel IN1
const int R2   = 23; // Right rear wheel IN2
const int R3   = 24; // Right front wheel IN3
const int R4   = 25; // Right front wheel IN4
const int ENA1 = 5;  // PWM enable A (right rear)
const int ENB1 = 4;  // PWM enable B (right front)

const int L1   = 27; // Left front wheel IN1
const int L2   = 26; // Left front wheel IN2
const int L3   = 28; // Left rear wheel IN3
const int L4   = 29; // Left rear wheel IN4
const int ENA2 = 6;  // PWM enable A (left front)
const int ENB2 = 7;  // PWM enable B (left rear)

// --- 超音波感測器 ---
const int trigPin = 31;
const int echoPin = 30;

// --- 參數設定 ---
const int PWM_SPEED = 130;      // 馬達基準 PWM
const int ERR_TOL   = 20;       // 誤差容許值 (pixel)
const int DIST_SAFE = 20;       // 安全距離 (cm)
const unsigned long RX_TIMEOUT = 100; // ms 超時視為 N (no detect)

// --- 全域變數 ---
long lastRx = 0;
int errX = 0, errY = 0;
bool hasTarget = false;

// --- 馬達功能函式 ---
void setPWM(int v) {
  analogWrite(ENA1, v); analogWrite(ENB1, v);
  analogWrite(ENA2, v); analogWrite(ENB2, v);
}
void stopAll() {
  setPWM(0);
  digitalWrite(R1,LOW); digitalWrite(R2,LOW);
  digitalWrite(R3,LOW); digitalWrite(R4,LOW);
  digitalWrite(L1,LOW); digitalWrite(L2,LOW);
  digitalWrite(L3,LOW); digitalWrite(L4,LOW);
}
void forwardDrive() {
  // 你實測的 forward 方向
  digitalWrite(R1,HIGH); digitalWrite(R2,LOW);
  digitalWrite(R3,LOW);  digitalWrite(R4,HIGH);
  digitalWrite(L1,LOW);  digitalWrite(L2,HIGH);
  digitalWrite(L3,HIGH); digitalWrite(L4,LOW);
  setPWM(PWM_SPEED);
}
void reverseDrive() {
  digitalWrite(R1,LOW);  digitalWrite(R2,HIGH);
  digitalWrite(R3,HIGH); digitalWrite(R4,LOW);
  digitalWrite(L1,HIGH); digitalWrite(L2,LOW);
  digitalWrite(L3,LOW);  digitalWrite(L4,HIGH);
  setPWM(PWM_SPEED);
}
void turnLeft() {
  // 右退、左進
  digitalWrite(R1,LOW);  digitalWrite(R2,HIGH);
  digitalWrite(R3,HIGH); digitalWrite(R4,LOW);
  digitalWrite(L1,LOW);  digitalWrite(L2,HIGH);
  digitalWrite(L3,HIGH); digitalWrite(L4,LOW);
  setPWM(PWM_SPEED);
}
void turnRight() {
  // 右進、左退
  digitalWrite(R1,HIGH); digitalWrite(R2,LOW);
  digitalWrite(R3,LOW);  digitalWrite(R4,HIGH);
  digitalWrite(L1,HIGH); digitalWrite(L2,LOW);
  digitalWrite(L3,LOW);  digitalWrite(L4,HIGH);
  setPWM(PWM_SPEED);
}

// --- 讀超音波距離 ---
int readDistance() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 30000);
  return (dur>0) ? dur*0.034/2 : 999;
}

void setup() {
  Serial.begin(57600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // 馬達腳位
  pinMode(R1,OUTPUT); pinMode(R2,OUTPUT);
  pinMode(R3,OUTPUT); pinMode(R4,OUTPUT);
  pinMode(ENA1,OUTPUT); pinMode(ENB1,OUTPUT);
  pinMode(L1,OUTPUT); pinMode(L2,OUTPUT);
  pinMode(L3,OUTPUT); pinMode(L4,OUTPUT);
  pinMode(ENA2,OUTPUT); pinMode(ENB2,OUTPUT);
  stopAll();
}

void loop() {
  // 1. 讀 Vision 訊息
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();  
    if (line.startsWith("E,")) {
      // E,errX,errY
      sscanf(line.c_str(), "E,%d,%d", &errX, &errY);
      hasTarget = true;  
      lastRx = millis();
    } else {
      hasTarget = false;
    }
  }
  // 超時
  if (millis() - lastRx > RX_TIMEOUT) {
    hasTarget = false;
  }

  // 2. 量超音波
  int dist = readDistance();

  // 3. 決策
  if (hasTarget && dist > DIST_SAFE) {
    // 先水平對準
    if      (errX >  ERR_TOL) turnLeft();
    else if (errX < -ERR_TOL) turnRight();
    // 水平 OK 再垂直
    else if (errY >  ERR_TOL) reverseDrive();
    else if (errY < -ERR_TOL) forwardDrive();
    else                       stopAll();
  } else {
    stopAll();
  }

  delay(20);
}
