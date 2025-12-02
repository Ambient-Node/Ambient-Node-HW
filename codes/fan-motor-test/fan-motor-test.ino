#include <SAMD21turboPWM.h>

TurboPWM turboPwm;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("=== TurboPWM test ===");
  
  // 1kHz로 테스트
  turboPwm.setClockDivider(1, false);
  turboPwm.timer(0, 1, 48000, true);
  
  Serial.print("주파수: ");
  Serial.println(turboPwm.frequency(0));
  
  // 듀티 0% → 100%
  for (int duty = 0; duty <= 1000; duty += 100) {
    turboPwm.analogWrite(8, duty);
    Serial.print("Duty: ");
    Serial.print(duty);
    Serial.println("/1000");
    delay(2000);
  }
}

void loop() {}
