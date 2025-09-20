#include <Ultrasonic.h>
Ultrasonic ultrasonic(12, 13);
int distance;
const int stopDistance = 10;   // 當距離小於等於 stopDistance 公分時停止
const int resumeDistance = 10.5; // 當距離大於等於 resumeDistance 公分時繼續操作
const int motorSpeed = 150;    // 前進速度（0-255）
const int reverseSpeed = 150;  // 後退速度（0-255）
const int turnTime = 500;      // 右轉時間（毫秒）(決定右轉角度)
const int MT_R1 = 3; // 右側馬達正向端口
const int MT_R2 = 2; // 右側馬達反向端口
const int MT_L1 = 5; // 左側馬達正向端口
const int MT_L2 = 4; // 左側馬達反向端口
void setup() {
  Serial.begin(9600);
  pinMode(MT_R1, OUTPUT);
  pinMode(MT_R2, OUTPUT);
  pinMode(MT_L1, OUTPUT);
  pinMode(MT_L2, OUTPUT);
  forwardMotors();// 預設開始前進
}
void stopMotors() {
  analogWrite(MT_R1, 0);
  analogWrite(MT_R2, 0);
  analogWrite(MT_L1, 0);
  analogWrite(MT_L2, 0);
}
void forwardMotors() {
  analogWrite(MT_R1, motorSpeed);  // 右側馬達正向運轉
  analogWrite(MT_R2, 0);           // 右側馬達反向停止
  analogWrite(MT_L1, motorSpeed);  // 左側馬達正向運轉
  analogWrite(MT_L2, 0);           // 左側馬達反向停止
}
void reverseMotors() {
  analogWrite(MT_R1, 0);            // 右側馬達正向停止
  analogWrite(MT_R2, reverseSpeed); // 右側馬達反向運轉
  analogWrite(MT_L1, 0);            // 左側馬達正向停止
  analogWrite(MT_L2, reverseSpeed); // 左側馬達反向運轉
}
void turnRightMotors() {
  analogWrite(MT_R1, 0);           // 右側馬達停止
  analogWrite(MT_R2, 0);
  analogWrite(MT_L1, motorSpeed);  // 左側馬達正向運轉
  analogWrite(MT_L2, 0);
}
void loop() {
  // 讀取距離
  distance = ultrasonic.read(); // 讀取距離（公分）
  Serial.print("Distance in CM: ");
  Serial.println(distance);
  if (distance <= stopDistance && distance != 0) {// 檢測障礙物
    stopMotors();
    Serial.println("檢測到障礙物，停止前進。");
    reverseMotors();// 開始後退，直到達到安全距離
    while (true) {
      distance = ultrasonic.read();
      Serial.print("後退中，距離為：");
      Serial.println(distance);

      if (distance >= resumeDistance || distance == 0) {
        stopMotors();
        Serial.println("達到安全距離，停止後退。");
        break;
      }
      delay(100); // 延遲以防止讀取過快
    }
    turnRightMotors();// 右轉一段時間
    delay(turnTime); // 轉彎時間，調整以改變轉彎角度
    stopMotors();
    Serial.println("右轉完成，繼續前進。");
    // 重新開始前進
    forwardMotors();
  }
  delay(100); // 根據需要調整檢測頻率
}
