/*
  SG90 90° ↔ 60° 循環（由 'A' 觸發）
  -----------------------------------
  接線：
    橙 (Signal) → D9
    紅 (VCC)    → +5 V
    棕 (GND)    → GND
*/

#include <Servo.h>

Servo sg90;

const int ANGLE_HIGH = 90;     // 上端角度
const int ANGLE_LOW  = 60;     // 下端角度
const int STEP_DELAY = 15;     // 每 1° 的延遲 (ms)
const int HOLD_TIME  = 2000;   // 端點停留時間 (ms)

bool running     = false;      // 目前是否在循環
const bool continuous = true;  // true = 一直循環；false = 只跑一次

void setup() {
  Serial.begin(115200);
  sg90.attach(9);
  sg90.write(ANGLE_HIGH);      // 先停在 90°
  Serial.println("Ready. Type 'A' + Enter to start.");
}

void loop() {
  // ── 等待觸發指令 ─────────────────────
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'A' || c == 'a') {
      running = true;
      Serial.println("Start cycle!");
    }
  }

  // ── 若未觸發就什麼都不做 ────────────
  if (!running) return;

  // ── 90° → 60° ───────────────────────
  for (int pos = ANGLE_HIGH; pos >= ANGLE_LOW; pos--) {
    sg90.write(pos);
    Serial.println(pos);
    delay(STEP_DELAY);
  }
  delay(HOLD_TIME);            // 底端停 2 秒

  // ── 60° → 90° ───────────────────────
  for (int pos = ANGLE_LOW; pos <= ANGLE_HIGH; pos++) {
    sg90.write(pos);
    Serial.println(pos);
    delay(STEP_DELAY);
  }
  delay(HOLD_TIME);            // 頂端停 2 秒

  // 若只想跑一次，把 continuous 設成 false
  if (!continuous) running = false;
}
