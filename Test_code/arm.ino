#include <Servo.h>

// 建立伺服馬達物件
Servo motorA;  // SG90
Servo motorB;  // LD-220MG
Servo motorC;  // LD-220MG
Servo motorD;  // MG996R

// 當前與目標角度
int posA = 90, posB = 90, posC = 90, posD = 0;
int targetA = 90, targetB = 90, targetC = 90, targetD = 0;

// 控制步進速度（數值小＝轉快）
const int everyA = 25;
const int everyB = 15;
const int everyC = 15;
const int everyD = 15;

// 是否正在移動
bool movingA = false, movingB = false, movingC = false, movingD = false;

// 控制流程步驟
int stepNow = 0;

void setup() {
  Serial.begin(9600);

  // 初始角度同步
  posA = 90; posB = 90; posC = 90; posD = 0;
  targetA = posA;
  targetB = posB;
  targetC = posC;
  targetD = posD;

  // SG90 限制脈波範圍以增加穩定性
  motorA.attach(8, 544, 2400);
  motorB.attach(9);
  motorC.attach(10);
  motorD.attach(11);

  // 寫入當前角度，避免爆衝
  motorA.write(posA);
  motorB.write(posB);
  motorC.write(posC);
  motorD.write(posD);

  Serial.println("初始化完成，等待啟動...");
  delay(2000);
  Serial.println("開始動作流程");
}

void loop() {
  checkServoA();
  checkServoB();
  checkServoC();
  checkServoD();
  executeSequence();
}

// 控制動作流程
void executeSequence() {
  static unsigned long lastTime = 0;
  static bool waiting = false;

  switch (stepNow) {
    case 0:
      targetC = 180;
      movingC = true;
      stepNow++;
      break;

    case 1:
      if (!movingC && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        stepNow++;
      }
      break;

    case 2:
      targetB = 117;
      movingB = true;
      stepNow++;
      break;

    case 3:
      if (!movingB && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        stepNow++;
      }
      break;

    case 4:
      targetA = 20;
      movingA = true;
      stepNow++;
      break;

    case 5:
      if (!movingA && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;

        // ✅ 同時讓 C 回 90 度、B 轉 180 度
        targetC = 90;
        movingC = true;

        targetB = 180;
        movingB = true;

        stepNow++;
      }
      break;

    case 6:
      if (!movingC && !movingB && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetD = 190;
        movingD = true;
        stepNow++;
      }
      break;

    case 7:
      if (!movingD && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetC = 70;
        movingC = true;
        stepNow++;
      }
      break;

    case 8:
      if (!movingC && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetB = 180;
        movingB = true;
        stepNow++;
      }
      break;

    case 9:
      if (!movingB && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetA = 90;
        movingA = true;
        stepNow++;
      }
      break;

    case 10:
      if (!movingA && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetB = 90; movingB = true;
        targetC = 90; movingC = true;
        stepNow++;
      }
      break;

    case 11:
      if (!movingB && !movingC && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        targetD = 0;
        movingD = true;
        stepNow++;
      }
      break;

    case 12:
      if (!movingD && !waiting) {
        lastTime = millis();
        waiting = true;
      }
      if (waiting && millis() - lastTime >= 2000) {
        waiting = false;
        Serial.println("✅ 所有動作完成！");
        stepNow++;
      }
      break;
  }
}

// 各伺服馬達平滑控制
void checkServoA() {
  static uint32_t t = 0;
  if (millis() - t < everyA) return;
  t = millis();

  if (posA < targetA) posA++;
  else if (posA > targetA) posA--;
  else { movingA = false; return; }

  motorA.write(posA);
}

void checkServoB() {
  static uint32_t t = 0;
  if (millis() - t < everyB) return;
  t = millis();

  if (posB < targetB) posB++;
  else if (posB > targetB) posB--;
  else { movingB = false; return; }

  motorB.write(posB);
}

void checkServoC() {
  static uint32_t t = 0;
  if (millis() - t < everyC) return;
  t = millis();

  if (posC < targetC) posC++;
  else if (posC > targetC) posC--;
  else { movingC = false; return; }

  motorC.write(posC);
}

void checkServoD() {
  static uint32_t t = 0;
  if (millis() - t < everyD) return;
  t = millis();

  if (posD < targetD) posD++;
  else if (posD > targetD) posD--;
  else { movingD = false; return; }

  motorD.write(posD);
}
