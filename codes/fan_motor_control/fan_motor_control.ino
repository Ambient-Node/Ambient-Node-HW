#define PWM_PIN 9
#define DIR_PIN 7
#define FG_PIN 3

// 전역 변수를 맨 위에 선언
volatile unsigned long pulseCount = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(FG_PIN, INPUT_PULLUP);
  
  // Timer1설정 D9, D10 
  // 31.25kHz PWM생성
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 511;  // 31.25kHz (16MHz / 512)
  
  attachInterrupt(digitalPinToInterrupt(FG_PIN), countPulse, CHANGE);
  
  digitalWrite(DIR_PIN, HIGH);
  OCR1A = 0;  // 정지상태로시작
  
  Serial.println("Motor Ready - PWM at 31.25kHz on D9");
  Serial.println("Commands: s[0-255]=speed, d=direction");
}

void loop() {
  if(Serial.available() > 0) {
    char cmd = Serial.read();
    
    if(cmd == 's') {
      int speed = Serial.parseInt();
      speed = constrain(speed, 0, 255);
      
      // 0-255를 0-511로 변환
      OCR1A = map(speed, 0, 255, 0, 511);
      
      Serial.print("Speed: ");
      Serial.println(speed);
    }
    else if(cmd == 'd') {
      bool dir = !digitalRead(DIR_PIN);
      digitalWrite(DIR_PIN, dir);
      
      Serial.print("Direction: ");
      Serial.println(dir ? "CW" : "CCW");
    }
  }
  
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint >= 2000) {
    Serial.print("Pulses/sec: ");
    Serial.println(pulseCount / 2.0);
    
    pulseCount = 0;
    lastPrint = millis();
  }
}

void countPulse() {
  pulseCount++;
}
