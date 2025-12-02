#include <FlashStorage.h>


// 모터 1 핀 정의
#define STEP_PIN1 4
#define DIR_PIN1 3
#define ENABLE_PIN1 5


// 모터 2 핀 정의
#define STEP_PIN2 1
#define DIR_PIN2 10
#define ENABLE_PIN2 2


// 저장할 데이터 구조체
typedef struct {
  int motor2_position;
  int valid;
} MotorPosition;


// FlashStorage 객체 생성
FlashStorage(motor_flash_storage, MotorPosition);


// 모터 2 위치 추적
int motor2_position = 0;
const int HOME_POSITION = 0;
const int MAX_CCW_STEPS = 400;
const int VALID_SIGNATURE = 12345;


void setup() {
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
  
  Serial.begin(9600);
  Serial.setTimeout(50);
  delay(100);
  
  // FlashStorage에서 마지막 위치 복원
  MotorPosition savedData = motor_flash_storage.read();
  
  if (savedData.valid == VALID_SIGNATURE &&
      savedData.motor2_position >= HOME_POSITION && 
      savedData.motor2_position <= MAX_CCW_STEPS) {
    
    motor2_position = savedData.motor2_position;
    
    Serial.println("=== 2축 모터 제어 시스템 준비 완료 ===");
    Serial.println("플래시에서 위치 복원됨:");
    Serial.print("  현재 위치: 홈에서 CCW로 ");
    Serial.print(motor2_position);
    Serial.println(" 스텝");
  } else {
    motor2_position = HOME_POSITION;
    
    Serial.println("=== 2축 모터 제어 시스템 준비 완료 ===");
    Serial.println("홈 위치로 설정됨 (CW 끝 지점)");
    
    savePositionToFlash();
  }
  
  Serial.println("");
  Serial.println("위치 시스템:");
  Serial.println("  홈(0) = CW 끝 지점");
  Serial.println("  CCW 방향으로만 최대 400스텝 이동 가능");
  Serial.println("");
  Serial.println("속도 프로파일:");
  Serial.println("  시작 속도: 1500 μs (느림)");
  Serial.println("  최고 속도: 500 μs (빠름)");
  Serial.println("  속도 변화: 100 μs씩 점진적");
  Serial.println("");
  Serial.println("명령어:");
  Serial.println("  1 cw 100/200/300/...  - 모터1 CW (100단위)");
  Serial.println("  1 ccw 100/200/300/... - 모터1 CCW (100단위)");
  Serial.println("  2 cw/ccw 100/200/...  - 모터2 (100단위)");
  Serial.println("  home                  - 모터2 홈 복귀");
  Serial.println("  sethome               - 현재 위치를 홈으로 설정");
  Serial.println("  clear                 - 플래시 메모리 초기화");
  Serial.println("  motor-test            - 전체 모터 테스트 실행");
  Serial.println("");
  Serial.print("현재 위치: ");
  Serial.print(motor2_position);
  Serial.print(" / ");
  Serial.println(MAX_CCW_STEPS);
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
  
  Serial.println("✓ 플래시 메모리 초기화 완료");
  Serial.println("  위치가 홈(0)으로 리셋되었습니다.");
}


void disableAllMotors() {
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
}


// 100 단위로 점진적 속도 변화 (1500→500)
void stepMotor(int motorNum, int steps, int initialDelay) {
  int stepPin, enablePin;
  
  if (motorNum == 1) {
    stepPin = STEP_PIN1;
    enablePin = ENABLE_PIN1;
  } else {
    stepPin = STEP_PIN2;
    enablePin = ENABLE_PIN2;
  }
  
  // 모터 활성화
  digitalWrite(enablePin, LOW);
  delayMicroseconds(500);
  
  // 속도 설정
  const int START_SPEED = 1500;  // 시작 속도 (느림)
  const int MAX_SPEED = 500;     // 최고 속도 (빠름)
  const int SPEED_STEP = 100;    // 속도 변화량
  
  // 가속/감속 구간을 전체의 30%로 설정
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
      // 가속 구간 - 5차 곡선으로 부드러운 가속
      float ratio = (float)i / accelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      
      // 1500에서 500까지 100씩 변화
      currentDelay = START_SPEED - (START_SPEED - MAX_SPEED) * ratio;
      
      // 100 단위로 반올림
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
      
    } else if (i < accelSteps + constantSteps) {
      // 등속 구간
      currentDelay = MAX_SPEED;
      
    } else {
      // 감속 구간 - 5차 곡선으로 부드러운 감속
      int decelProgress = i - accelSteps - constantSteps;
      float ratio = (float)decelProgress / decelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      
      // 500에서 1500까지 100씩 변화
      currentDelay = MAX_SPEED + (START_SPEED - MAX_SPEED) * ratio;
      
      // 100 단위로 반올림
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
  }
  
  // 모터 비활성화 전 안정화
  delayMicroseconds(500);
  digitalWrite(enablePin, HIGH);
  delayMicroseconds(100);
}


void stepMotorSimultaneous(int steps1, bool dir1, int steps2, bool dir2) {
  // 두 모터를 동시에 움직이는 함수
  digitalWrite(ENABLE_PIN1, LOW);
  digitalWrite(ENABLE_PIN2, LOW);
  delayMicroseconds(500);
  
  // 방향 설정
  digitalWrite(DIR_PIN1, dir1 ? HIGH : LOW);
  digitalWrite(DIR_PIN2, dir2 ? HIGH : LOW);
  delayMicroseconds(10);
  
  int maxSteps = max(steps1, steps2);
  
  // 속도 설정
  const int START_SPEED = 1500;
  const int MAX_SPEED = 500;
  const int SPEED_STEP = 100;
  
  // 가속/감속 구간 30%
  int accelSteps = (maxSteps * 3) / 10;
  if (accelSteps < 80) accelSteps = 80;
  if (accelSteps > 400) accelSteps = 400;
  
  int decelSteps = accelSteps;
  int constantSteps = maxSteps - accelSteps - decelSteps;
  
  if (constantSteps < 0) {
    accelSteps = maxSteps / 2;
    decelSteps = maxSteps - accelSteps;
    constantSteps = 0;
  }
  
  for(int i = 0; i < maxSteps; i++) {
    int currentDelay;
    
    if (i < accelSteps) {
      // 가속 구간
      float ratio = (float)i / accelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = START_SPEED - (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
      
    } else if (i < accelSteps + constantSteps) {
      // 등속 구간
      currentDelay = MAX_SPEED;
      
    } else {
      // 감속 구간
      int decelProgress = i - accelSteps - constantSteps;
      float ratio = (float)decelProgress / decelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = MAX_SPEED + (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    }
    
    // 모터1 스텝
    if (i < steps1) {
      digitalWrite(STEP_PIN1, HIGH);
    }
    
    // 모터2 스텝
    if (i < steps2) {
      digitalWrite(STEP_PIN2, HIGH);
    }
    
    delayMicroseconds(currentDelay);
    
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    
    delayMicroseconds(currentDelay);
  }
  
  delayMicroseconds(500);
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
  delayMicroseconds(100);
}


void motorTest() {
  Serial.println("");
  Serial.println("========================================");
  Serial.println("         모터 테스트 시작");
  Serial.println("========================================");
  Serial.println("");
  
  delay(500);
  
  Serial.println("[ 모터 개별 테스트 ]");
  Serial.println("");
  
  // --- 모터2 테스트 ---
  Serial.println(">> 모터2 테스트");
  
  if (motor2_position > 0) {
    Serial.print("   홈으로 복귀 중... (CW ");
    Serial.print(motor2_position);
    Serial.println(" 스텝)");
    digitalWrite(DIR_PIN2, HIGH);
    delayMicroseconds(10);
    stepMotor(2, motor2_position, 500);
    motor2_position = HOME_POSITION;
    delay(500);
  }
  
  Serial.println("   홈(0) → CCW 400 스텝");
  digitalWrite(DIR_PIN2, LOW);
  delayMicroseconds(10);
  stepMotor(2, 400, 500);
  motor2_position = 400;
  delay(1000);
  
  Serial.println("   CCW 400 → 홈(0) 복귀");
  digitalWrite(DIR_PIN2, HIGH);
  delayMicroseconds(10);
  stepMotor(2, 400, 500);
  motor2_position = HOME_POSITION;
  delay(1000);
  
  Serial.println("✓ 모터2 테스트 완료");
  Serial.println("");
  
  // --- 모터1 테스트 ---
  Serial.println(">> 모터1 테스트");
  
  Serial.println("   CW 500 스텝");
  digitalWrite(DIR_PIN1, HIGH);
  delayMicroseconds(10);
  stepMotor(1, 500, 500);
  delay(1000);
  
  Serial.println("   CCW 500 스텝 (원위치)");
  digitalWrite(DIR_PIN1, LOW);
  delayMicroseconds(10);
  stepMotor(1, 500, 500);
  delay(1000);
  
  Serial.println("✓ 모터1 테스트 완료");
  Serial.println("");
  
  // --- 동시 테스트 ---
  Serial.println("[ 모터 동시 테스트 ]");
  Serial.println("");
  
  if (motor2_position > 0) {
    digitalWrite(DIR_PIN2, HIGH);
    delayMicroseconds(10);
    stepMotor(2, motor2_position, 500);
    motor2_position = HOME_POSITION;
    delay(500);
  }
  
  Serial.println(">> 동시 이동 1");
  Serial.println("   모터1: CW 200 | 모터2: CCW 200");
  stepMotorSimultaneous(200, true, 200, false);
  motor2_position = 200;
  delay(1000);
  
  Serial.println(">> 동시 이동 2");
  Serial.println("   모터1: CCW 200 | 모터2: CW 200 (홈 복귀)");
  stepMotorSimultaneous(200, false, 200, true);
  motor2_position = HOME_POSITION;
  delay(1000);
  
  Serial.println("✓ 동시 테스트 완료");
  Serial.println("");
  
  savePositionToFlash();
  
  Serial.println("========================================");
  Serial.println("         모터 테스트 완료");
  Serial.println("========================================");
  Serial.println("");
  
  disableAllMotors();
}


void loop() {
  disableAllMotors();
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // "motor-test" 명령
    if (input == "motor-test") {
      Serial.println("모터 테스트를 시작합니다...");
      delay(500);
      motorTest();
      return;
    }
    
    // "clear" 명령
    if (input == "clear") {
      Serial.println("플래시 메모리를 초기화합니다...");
      clearFlashMemory();
      return;
    }
    
    // "sethome" 명령
    if (input == "sethome") {
      Serial.print("현재 위치(");
      Serial.print(motor2_position);
      Serial.println(")를 홈으로 설정합니다...");
      
      motor2_position = HOME_POSITION;
      savePositionToFlash();
      
      Serial.println("✓ 홈 위치 설정 완료");
      return;
    }
    
    // "home" 명령
    if (input == "home") {
      Serial.println("홈 위치로 복귀 중...");
      
      if (motor2_position > 0) {
        digitalWrite(DIR_PIN2, HIGH);
        delayMicroseconds(10);
        stepMotor(2, motor2_position, 500);
        
        motor2_position = HOME_POSITION;
        savePositionToFlash();
        
        Serial.println("✓ 홈 위치 도달");
      } else {
        Serial.println("이미 홈 위치입니다.");
      }
      
      return;
    }
    
    Serial.print("받은 명령: [");
    Serial.print(input);
    Serial.println("]");
    
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String motorStr = input.substring(0, firstSpace);
      String direction = input.substring(firstSpace + 1, secondSpace);
      String stepsStr = input.substring(secondSpace + 1);
      
      int motorNum = motorStr.toInt();
      int steps = stepsStr.toInt();
      direction.trim();
      
      // 100 단위 체크
      if (steps % 100 != 0) {
        Serial.print("✗ 스텝 수는 100 단위여야 합니다 (입력: ");
        Serial.print(steps);
        Serial.println(")");
        return;
      }
      
      if ((motorNum == 1 || motorNum == 2) && 
          (direction == "cw" || direction == "ccw") && 
          steps > 0) {
        
        // 모터 2번 제한 검사
        if (motorNum == 2) {
          if (direction == "ccw") {
            int new_position = motor2_position + steps;
            if (new_position > MAX_CCW_STEPS) {
              Serial.print("✗ CCW 제한 초과! (현재: ");
              Serial.print(motor2_position);
              Serial.print(", 이동 후: ");
              Serial.print(new_position);
              Serial.print(", 최대: ");
              Serial.print(MAX_CCW_STEPS);
              Serial.println(")");
              return;
            }
          } else {
            int new_position = motor2_position - steps;
            if (new_position < HOME_POSITION) {
              Serial.print("✗ 홈을 지나칠 수 없습니다! (현재: ");
              Serial.print(motor2_position);
              Serial.print(", 이동 후: ");
              Serial.print(new_position);
              Serial.println(")");
              return;
            }
          }
        }
        
        // 방향 설정 후 딜레이
        if (motorNum == 1) {
          digitalWrite(DIR_PIN1, (direction == "cw") ? HIGH : LOW);
          delayMicroseconds(10);
        } else {
          digitalWrite(DIR_PIN2, (direction == "cw") ? HIGH : LOW);
          delayMicroseconds(10);
        }
        
        // 모터 구동
        stepMotor(motorNum, steps, 500);
        
        // 모터 2번 위치 업데이트
        if (motorNum == 2) {
          if (direction == "ccw") {
            motor2_position += steps;
          } else {
            motor2_position -= steps;
          }
          
          savePositionToFlash();
          
          Serial.print("[모터2] 위치: ");
          Serial.print(motor2_position);
          Serial.print(" / ");
          Serial.println(MAX_CCW_STEPS);
        }
        
        Serial.println("✓ 완료");
        disableAllMotors();
      }
    }
  }
  
  delay(10);
}
