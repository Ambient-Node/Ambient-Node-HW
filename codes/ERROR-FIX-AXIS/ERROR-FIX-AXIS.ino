#include <EEPROM.h>

// ===== 핀 정의 =====
#define STEP_PIN2 D4
#define DIR_PIN2 D3
#define ENABLE_PIN2 D5

#define STEP_PIN1 D1
#define DIR_PIN1 D10
#define ENABLE_PIN1 D2

#define BLDC_PWM_PIN D8

void setup() {
  // 핀리셋
  for (int i = 0; i <= 10; i++) {
    pinMode(i, INPUT);
  }
  delay(200);
  
  // BLDC 정지
  pinMode(BLDC_PWM_PIN, OUTPUT);
  digitalWrite(BLDC_PWM_PIN, HIGH);
  delay(100);
  
  Serial.begin(9600);
  Serial.setTimeout(10);
  delay(500);
  
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
  
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);
  
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);
  
  Serial.println("=== 스테퍼 모터 1,2 테스트 (PWM   정지) ===");
  Serial.println("1 cw 100");
  Serial.println("1 ccw 100");
  Serial.println("2 cw 100");
  Serial.println("2 ccw 100");
  Serial.println("Ready!");
}

void stepMotor(int motorNum, int steps) {
  int stepPin, dirPin, enablePin;
  
  if (motorNum == 1) {
    stepPin = STEP_PIN1;
    dirPin = DIR_PIN1;
    enablePin = ENABLE_PIN1;
  } else {
    stepPin = STEP_PIN2;
    dirPin = DIR_PIN2;
    enablePin = ENABLE_PIN2;
  }
  
  digitalWrite(enablePin, LOW);
  delayMicroseconds(1000);
  
  const int START_SPEED = 1500;
  const int MAX_SPEED = 500;
  const int SPEED_STEP = 100;
  
  int accelSteps = (steps * 3) / 10;
  if (accelSteps < 80) accelSteps = 80;
  if (accelSteps > 400) accelSteps = 400;
  
  int decelSteps = accelSteps;
  int constantSteps = steps - accelSteps - decelSteps;
  
  if (constantSteps < 0) {
    accelSteps = steps / 2;
    decelSteps = steps - accelSteps;
    constantSteps = 0;
  }
  
  for (int i = 0; i < steps; i++) {
    int currentDelay;
    
    if (i < accelSteps) {
      float ratio = (float)i / accelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = START_SPEED - (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    } else if (i < accelSteps + constantSteps) {
      currentDelay = MAX_SPEED;
    } else {
      int decelProgress = i - accelSteps - constantSteps;
      float ratio = (float)decelProgress / decelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = MAX_SPEED + (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
  }
  
  delayMicroseconds(500);
  digitalWrite(enablePin, HIGH);
  delay(1);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      int motorNum = input.substring(0, firstSpace).toInt();
      String direction = input.substring(firstSpace + 1, secondSpace);
      int steps = input.substring(secondSpace + 1).toInt();
      direction.trim();
      
      if (steps % 100 != 0) {
        Serial.println("✗ 100단위");
        return;
      }
      
      if ((motorNum == 1 || motorNum == 2) &&
          (direction == "cw" || direction == "ccw") &&
          steps > 0) {
        
        if (motorNum == 1) {
          digitalWrite(DIR_PIN1, (direction == "cw") ? HIGH : LOW);
        } else {
          digitalWrite(DIR_PIN2, (direction == "cw") ? HIGH : LOW);
        }
        delayMicroseconds(10);
        
        stepMotor(motorNum, steps);
        
        Serial.println("✓");
      }
    }
  }
  
  delay(5);
}
