#include <FlashStorage.h>

// ===== 핀 정의 =====
#define STEP_PIN1 4
#define DIR_PIN1 3
#define ENABLE_PIN1 5

#define STEP_PIN2 1
#define DIR_PIN2 10
#define ENABLE_PIN2 2

#define BLDC_PWM_PIN 8
#define BLDC_FG_PIN 9  // FG 핀 추가(bldc rpm)

// ===== 변수 =====
typedef struct {
  int motor2_position;
  int valid;
} MotorPosition;

FlashStorage(motor_flash_storage, MotorPosition);

int motor2_position = 0;
const int HOME_POSITION = 0;
const int MAX_CCW_STEPS = 400;
const int VALID_SIGNATURE = 12345;

int bldcSpeedLevel = 0;

// 임시 5단
const int SPEED_LEVELS[6] = {255, 220, 180, 140, 100, 0};

// RPM 측정용 변수
volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

// FG 인터럽트 함수
void countFgPulse() {
  fgPulseCount++;
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  delay(500);
  
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(ENABLE_PIN1, OUTPUT);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(ENABLE_PIN1, HIGH);
  
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);
  digitalWrite(ENABLE_PIN2, HIGH);
  
  // BLDC 초기화
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWrite(BLDC_PWM_PIN, 255);  // 정지
  
  // FG핀설정(인터럽트)
  pinMode(BLDC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BLDC_FG_PIN), countFgPulse, FALLING);
  
  MotorPosition savedData = motor_flash_storage.read();
  
  if (savedData.valid == VALID_SIGNATURE &&
      savedData.motor2_position >= HOME_POSITION && 
      savedData.motor2_position <= MAX_CCW_STEPS) {
    motor2_position = savedData.motor2_position;
    Serial.println("=== 3축 시스템 (RPM 측정) ===");
    Serial.print("위치: ");
    Serial.println(motor2_position);
  } else {
    motor2_position = HOME_POSITION;
    Serial.println("=== 3축 시스템 (RPM 측정) ===");
    savePositionToFlash();
  }
  
  Serial.println("");
  Serial.println("[ 스테퍼 모터 ]");
  Serial.println("  1 cw/ccw 100/200/...");
  Serial.println("  2 cw/ccw 100/200/...");
  Serial.println("  속도: 1500→500μs");
  Serial.println("");
  Serial.println("[ BLDC 팬 ]");
  Serial.println("  3 speed 0~5");
  Serial.println("  0=정지, 1=최소, 5=최대");
  Serial.println("  RPM 자동 측정 (FG 핀)");
  Serial.println("");
  Serial.println("명령: home / sethome / clear");
  Serial.println("Ready!");
  
  lastRpmTime = millis();
}

void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  motor_flash_storage.write(dataToSave);
}

void clearFlashMemory() {
  MotorPosition emptyData;
  emptyData.motor2_position = HOME_POSITION;
  emptyData.valid = 0;
  motor_flash_storage.write(emptyData);
  motor2_position = HOME_POSITION;
  Serial.println("✓");
}

void disableAllMotors() {
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
}

void stepMotor(int motorNum, int steps, int initialDelay) {
  int stepPin, enablePin;
  
  if (motorNum == 1) {
    stepPin = STEP_PIN1;
    enablePin = ENABLE_PIN1;
  } else {
    stepPin = STEP_PIN2;
    enablePin = ENABLE_PIN2;
  }
  
  digitalWrite(enablePin, LOW);
  delayMicroseconds(500);
  
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
  
  for(int i = 0; i < steps; i++) {
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
  delayMicroseconds(100);
}

void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  const int pwmValues[6] = {255, 235, 220, 200, 100, 0};
  int targetPwm = pwmValues[level];
  
  // 정지
  if (level == 0) {
    int currentPwm = pwmValues[bldcSpeedLevel];
    
    if (currentPwm < 140) {
      for (int pwm = currentPwm; pwm <= 140; pwm += 20) {
        analogWrite(BLDC_PWM_PIN, pwm);
        delay(10);
      }
    }
    
    for (int pwm = (currentPwm < 140 ? 140 : currentPwm); pwm <= 255; pwm += 10) {
      if (pwm > 255) pwm = 255;
      analogWrite(BLDC_PWM_PIN, pwm);
      delay(30);
    }
    
    analogWrite(BLDC_PWM_PIN, 255);
    bldcSpeedLevel = 0;
    currentRPM = 0;
    Serial.println("✓ BLDC 정지");
    return;
  }
  
  // 재시작-정지 상태에서 시작
  if (bldcSpeedLevel == 0) {
    pinMode(BLDC_PWM_PIN, INPUT);
    delay(100);
    pinMode(BLDC_PWM_PIN, OUTPUT);
    delay(50);
    
    analogWrite(BLDC_PWM_PIN, 150);
    delay(100);
    
    for (int pwm = 150; pwm >= 140; pwm -= 10) {
      analogWrite(BLDC_PWM_PIN, pwm);
      delay(10);
    }
    
    for (int pwm = 140; pwm >= targetPwm; pwm -= 10) {
      if (pwm < targetPwm) pwm = targetPwm;
      analogWrite(BLDC_PWM_PIN, pwm);
      delay(40);
    }
    
    // RPM 카운터 리셋
    fgPulseCount = 0;
    lastRpmTime = millis();
  } 
  else {
    int currentPwm = pwmValues[bldcSpeedLevel];
    
    if (currentPwm < targetPwm) {
      if (currentPwm < 140 && targetPwm > 140) {
        for (int pwm = currentPwm; pwm <= 140; pwm += 10) {
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(30);
        }
        for (int pwm = 140; pwm <= targetPwm; pwm += 20) {
          if (pwm > targetPwm) pwm = targetPwm;
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(10);
        }
      } else {
        for (int pwm = currentPwm; pwm <= targetPwm; pwm += 10) {
          if (pwm > targetPwm) pwm = targetPwm;
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(30);
        }
      }
    }
    else if (currentPwm > targetPwm) {
      if (currentPwm > 140 && targetPwm < 140) {
        for (int pwm = currentPwm; pwm >= 140; pwm -= 20) {
          if (pwm < 140) pwm = 140;
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(10);
        }
        for (int pwm = 140; pwm >= targetPwm; pwm -= 10) {
          if (pwm < targetPwm) pwm = targetPwm;
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(30);
        }
      } else {
        for (int pwm = currentPwm; pwm >= targetPwm; pwm -= 10) {
          if (pwm < targetPwm) pwm = targetPwm;
          analogWrite(BLDC_PWM_PIN, pwm);
          delay(30);
        }
      }
    }
  }
  
  analogWrite(BLDC_PWM_PIN, targetPwm);
  bldcSpeedLevel = level;
  
  Serial.print("✓ BLDC ");
  Serial.print(level);
  Serial.print(" (PWM=");
  Serial.print(targetPwm);
  Serial.println(")");
}

void measureRPM() {
  unsigned long currentTime = millis();
  
  // 1초마다 측정
  if (currentTime - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      // FG 펄스/회전 = 3으로 수정
      currentRPM = (fgPulseCount * 60) / 3;  // ← 2에서 3으로 변경
      
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
  disableAllMotors();
  
  // RPM 측정 (모터 가동 중일 때만)
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
      Serial.println("ok");
      return;
    }
    
    if (input == "home") {
      if (motor2_position > 0) {
        digitalWrite(DIR_PIN2, HIGH);
        delayMicroseconds(10);
        stepMotor(2, motor2_position, 500);
        motor2_position = HOME_POSITION;
        savePositionToFlash();
        Serial.println("ok");
      }
      return;
    }
    
    if (input.startsWith("3 speed ")) {
      int level = input.substring(8).toInt();
      if (level >= 0 && level <= 5) {
        setBldcSpeed(level);
      } else {
        Serial.println("0~5");
      }
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
        Serial.println("100단위");
        return;
      }
      
      if ((motorNum == 1 || motorNum == 2) && 
          (direction == "cw" || direction == "ccw") && 
          steps > 0) {
        
        if (motorNum == 2) {
          if (direction == "ccw" && motor2_position + steps > MAX_CCW_STEPS) {
            Serial.println("제한");
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
        
        Serial.println("ok");
      }
    }
  }
  
  delay(5);
}
