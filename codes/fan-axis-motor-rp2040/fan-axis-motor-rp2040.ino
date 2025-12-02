#include <EEPROM.h>

// XIAO PWM 문제로  XIAO RP2040으로 컨트롤러 변경

// ===== 핀 정의 =====
#define STEP_PIN2 D4
#define DIR_PIN2 D3
#define ENABLE_PIN2 D5

#define STEP_PIN1 D1
#define DIR_PIN1 D10
#define ENABLE_PIN1 D2

#define BLDC_PWM_PIN D8
#define BLDC_FG_PIN D9

// ===== 변수 =====
typedef struct {
  int motor2_position;
  int valid;
} MotorPosition;

int motor2_position = 0;
const int HOME_POSITION = 0;
const int MAX_CCW_STEPS = 300;
const int VALID_SIGNATURE = 12345;

int bldcSpeedLevel = 0;
int currentBldcPwm = 255;

const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

void countFgPulse() {
  fgPulseCount++;
}

// === 20단계 PWM 테스트 함수 ===
void fanTest() {
  Serial.println("=== fan-test: PWM 200~70 20단계 테스트 시작 ===");
  Serial.println("진동O, 무진동X");
  
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);

  int acceptedValues[20];
  int acceptedCount = 0;
  

  int startPwm = 200;
  if (currentBldcPwm > startPwm) {
    for (int pwm = currentBldcPwm; pwm >= startPwm; pwm -= 5) {
      analogWrite(BLDC_PWM_PIN, pwm);
      currentBldcPwm = pwm;
      delay(20);
    }
  } else if (currentBldcPwm < startPwm) {
    for (int pwm = currentBldcPwm; pwm <= startPwm; pwm += 5) {
      analogWrite(BLDC_PWM_PIN, pwm);
      currentBldcPwm = pwm;
      delay(20);
    }
  }
  
  for (int i = 0; i < 20; i++) {
    int targetPwm = 200 - i * ((200 - 70) / 19);
    analogWrite(BLDC_PWM_PIN, targetPwm);
    currentBldcPwm = targetPwm;
    
    Serial.print("테스트 PWM = ");
    Serial.print(targetPwm);
    Serial.print(" (듀티 ");
    Serial.print((255 - targetPwm) * 100 / 255);
    Serial.println("%) "진동O, 무진동X: ");
    
    while (true) {
      if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        if (input == "o") {
          acceptedValues[acceptedCount++] = targetPwm;
          Serial.println("ok");
          break;
        } else if (input == "x") {
          Serial.println("not ok");
          break;
        } else {
          Serial.println("input err");
        }
      }
      delay(10);
    }
  }
  
  // 테스트종료 후 팬정지
  while (currentBldcPwm < 255) {
    currentBldcPwm++;
    analogWrite(BLDC_PWM_PIN, currentBldcPwm);
    delay(10);
  }
  bldcSpeedLevel = 0;
  
  Serial.println("=== fan-test 완료 ===");
  Serial.print("안정적인 PWM 값: ");
  for (int i = 0; i < acceptedCount; i++) {
    Serial.print(acceptedValues[i]);
    if (i < acceptedCount - 1) Serial.print(", ");
  }
  Serial.println();
}




void setup() {
  pinMode(BLDC_PWM_PIN, OUTPUT);
  digitalWrite(BLDC_PWM_PIN, HIGH);
  delay(50);
  
  Serial.begin(9600);
  Serial.setTimeout(10);
  delay(500);
  
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
  delay(10);
  
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);
  
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);
  
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255);
  currentBldcPwm = 255;
  
  pinMode(BLDC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BLDC_FG_PIN), countFgPulse, RISING);
  
  EEPROM.begin(512);
  MotorPosition savedData;
  EEPROM.get(0, savedData);
  
  if (savedData.valid == VALID_SIGNATURE &&
      savedData.motor2_position >= HOME_POSITION &&
      savedData.motor2_position <= MAX_CCW_STEPS) {
    motor2_position = savedData.motor2_position;
    Serial.println("=== 3축 시스템 (RP2040, 30kHz) ===");
    Serial.print("위치: ");
    Serial.println(motor2_position);
  } else {
    motor2_position = HOME_POSITION;
    Serial.println("=== 3축 시스템 (RP2040, 30kHz) ===");
    savePositionToFlash();
  }
  
  Serial.println("");
  Serial.println("[ 스테퍼 모터 ]");
  Serial.println("  1 cw/ccw 100/200/...");
  Serial.println("  2 cw/ccw 100/200/...");
  Serial.println("");
  Serial.println("[ BLDC 팬 ]");
  Serial.println("  3 speed 0~5, fan-test 명령 추가됨");
  Serial.println("  (부팅 시 안전 정지)");
  Serial.println("");
  Serial.println("명령: home / sethome / clear / fan-test");
  Serial.println("Ready!");
  
  lastRpmTime = millis();
}

void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  EEPROM.put(0, dataToSave);
  EEPROM.commit();
  delay(10);
}

void clearFlashMemory() {
  MotorPosition emptyData;
  emptyData.motor2_position = HOME_POSITION;
  emptyData.valid = 0;
  EEPROM.put(0, emptyData);
  EEPROM.commit();
  delay(10);
  motor2_position = HOME_POSITION;
  Serial.println("✓");
}

void stepMotor(int motorNum, int steps, int initialDelay) {
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
  
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, currentBldcPwm);
}

void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  int targetPwm = SPEED_LEVELS[level];
  
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  
  if (currentBldcPwm != targetPwm) {
    int step = (currentBldcPwm < targetPwm) ? +1 : -1;
    while (currentBldcPwm != targetPwm) {
      currentBldcPwm += step;
      analogWrite(BLDC_PWM_PIN, currentBldcPwm);
      delay(15);
    }
  }
  
  bldcSpeedLevel = level;
  
  Serial.print("✓ BLDC 속도 ");
  Serial.print(level);
  Serial.print(" (PWM=");
  Serial.print(targetPwm);
  Serial.println(")");
}


void measureRPM() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3;
      Serial.print("[RPM] ");
      Serial.print(currentRPM);
      Serial.print(" (레벨 ");
      Serial.print(bldcSpeedLevel);
      Serial.println(")");
      fgPulseCount = 0;
    }
    lastRpmTime = currentTime;
  }
}

void loop() {
  if (bldcSpeedLevel > 0) {
    measureRPM();
  }
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "clear") {
      clearFlashMemory();
      return;
    }
    
    if (input == "sethome") {
      motor2_position = HOME_POSITION;
      savePositionToFlash();
      Serial.println("✓");
      return;
    }
    
    if (input == "home") {
      if (motor2_position > 0) {
        digitalWrite(DIR_PIN2, HIGH);
        delayMicroseconds(10);
        stepMotor(2, motor2_position, 500);
        motor2_position = HOME_POSITION;
        savePositionToFlash();
        Serial.println("✓");
      }
      return;
    }
    
    if (input.startsWith("3 speed ")) {
      int level = input.substring(8).toInt();
      if (level >= 0 && level <= 5) {
        setBldcSpeed(level);
      } else {
        Serial.println("✗ 0~5");
      }
      return;
    }
    
    // fan-test 명령어 추가
    if (input == "fan-test") {
      fanTest();
      return;
    }
    
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
        
        if (motorNum == 2) {
          if (direction == "ccw" && motor2_position + steps > MAX_CCW_STEPS) {
            Serial.println("✗ 제한");
            return;
          }
          if (direction == "cw" && motor2_position - steps < HOME_POSITION) {
            Serial.println("✗ 홈 초과");
            return;
          }
        }
        
        if (motorNum == 1) {
          digitalWrite(DIR_PIN1, (direction == "cw") ? HIGH : LOW);
        } else {
          digitalWrite(DIR_PIN2, (direction == "cw") ? HIGH : LOW);
        }
        delayMicroseconds(10);
        
        stepMotor(motorNum, steps, 500);
        
        if (motorNum == 2) {
          if (direction == "ccw") {
            motor2_position += steps;
          } else {
            motor2_position -= steps;
          }
          savePositionToFlash();
        }
        
        Serial.println("✓");
      }
    }
  }
  
  delay(5);
}
