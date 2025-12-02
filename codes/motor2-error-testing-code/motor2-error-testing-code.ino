// [진단용] 모터 2번 마이크로스텝 설정 확인 코드


#define STEP_PIN2 D4
#define DIR_PIN2 D3
#define ENABLE_PIN2 D5

void setup() {
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN2, LOW); // 활성화

  Serial.begin(9600);
  Serial.println("Motor 2 Slow Test Start");
}

void loop() {
  digitalWrite(DIR_PIN2, HIGH);

  Serial.println("Moving...");
  for(int i = 0; i < 200; i++) { 
    digitalWrite(STEP_PIN2, HIGH);
    
    delayMicroseconds(2000); 
    
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(2000); 
  }
  
  delay(1000);
}