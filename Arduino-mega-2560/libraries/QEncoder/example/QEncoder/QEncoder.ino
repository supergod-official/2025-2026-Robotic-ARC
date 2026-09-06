// 例：使用 PCINT 组0（PORTB）的四对引脚做 4 个电机编码器
// (D53,D52)  (D51,D50)  (D10,D11)  (D12,D13)
#include "QEncoder.h"
QEncoder enc0(3, 52);
//QEncoder enc1(51, 50);
//QEncoder enc2(10, 11);
//QEncoder enc3(12, 13);

void setup() {
  Serial.begin(115200);

  // 注意：D10..D13 同时是 SPI 引脚。如果在用 SPI，请避免把它们用作编码器，
  // 或不要在 PCMSK0 里开启对应位（改用 A8..A15 的 PORTK 更安全）。
  enc0.begin(true);
  //enc1.begin(true);
  //enc2.begin(true);
  //enc3.begin(true);
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    Serial.print("E0="); Serial.println(enc0.read());
    //Serial.print("  E1="); Serial.print(enc1.read());
    //Serial.print("  E2="); Serial.print(enc2.read());
    //Serial.print("  E3="); Serial.println(enc3.read());
  }
}
