#include <EEPROM.h>

// ===== 핀 정의 =====
// 모터2 (틸트)
#define STEP_PIN2 D4
#define DIR_PIN2 D3
#define ENABLE_PIN2 D5

// 모터1 (팬 좌우)
#define STEP_PIN1 D1
#define DIR_PIN1 D10
#define ENABLE_PIN1 D2

// BLDC 팬
#define BLDC_PWM_PIN D8
#define BLDC_FG_PIN D9

// ===== 위치 저장용 구조체 =====
typedef struct {
  int motor2_position;   // 모터2 현재 위치
  int valid;             // EEPROM 데이터 유효 플래그
} MotorPosition;

// ===== 위치 / 한계 값 =====
int motor2_position = 0;         // 모터2 현재 위치 카운터
const int HOME_POSITION = 0;     // 모터2 홈 위치
const int MAX_CCW_STEPS = 300;   // 모터2 CCW 최대 스텝
const int VALID_SIGNATURE = 12345; // EEPROM 데이터 유효 시그니처

// ===== BLDC 제어 변수 =====
int bldcSpeedLevel = 0;          // BLDC 속도 레벨 (0~5)
int currentBldcPwm = 255;        // 현재 PWM 출력 값
// 각 속도 레벨별 PWM 값
const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

// ===== BLDC RPM 측정 =====
volatile unsigned long fgPulseCount = 0; // FG 인터럽트 펄스 카운트
unsigned long lastRpmTime = 0;           // 마지막 RPM 계산 시각
int currentRPM = 0;                      // 계산된 RPM 값

// ===== UART 모터 제어 변수 =====
// 모터1 위치 및 한계
int motor1_position = 0;          // 모터1 위치 카운터
const int MOTOR1_CW_LIMIT = -300; // CW 방향 최소 위치
const int MOTOR1_CCW_LIMIT = 300; // CCW 방향 최대 위치

// UART 기반 모터1/2 동작 상태
bool motor1_moving = false;
bool motor2_moving_uart = false;
int motor1_direction = 0;         // 0: 정지, 1: CW, 2: CCW
int motor2_direction_uart = 0;    // 0: 정지, 1: CW(down), 2: CCW(up)

// UART 제어 모터 스텝 간 딜레이
const int UART_MOTOR_DELAY = 1000;

// FG 인터럽트 콜백 (BLDC 속도 측정용)
void countFgPulse() {
  fgPulseCount++;
}

// BLDC 팬 테스트 루틴 (수동으로 안정 PWM 구간 탐색)
void fanTest() {
  Serial.println("=== fan-test: PWM 200~70 20단계 테스트 시작 ===");
  Serial.println("떨림 없으면 'o' 입력, 떨림 있으면 'x' 입력 후 엔터");
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);

  int acceptedValues[20]; // 안정적이라고 판단된 PWM 저장
  int acceptedCount = 0;

  int startPwm = 200;
  // 현재 PWM에서 시작 PWM까지 부드럽게 이동
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

  // 200→70 구간을 20단계로 테스트
  for (int i = 0; i < 20; i++) {
    int targetPwm = 200 - i * ((200 - 70) / 19);
    analogWrite(BLDC_PWM_PIN, targetPwm);
    currentBldcPwm = targetPwm;

    Serial.print("테스트 PWM = ");
    Serial.print(targetPwm);
    Serial.print(" (듀티 ");
    Serial.print((255 - targetPwm) * 100 / 255);
    Serial.println("%) -> 떨림 없으면 'o', 있으면 'x' 입력:");

    // 사용자에게 떨림 여부 입력받기
    while (true) {
      if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        if (input == "o") {
          acceptedValues[acceptedCount++] = targetPwm;
          Serial.println("✓ 안정점으로 기록됨");
          break;
        } else if (input == "x") {
          Serial.println("✗ 떨림 발생");
          break;
        } else {
          Serial.println("입력 오류 - 'o' 혹은 'x'만 입력하세요");
        }
      }
      delay(10);
    }
  }

  // 안전하게 정지 방향으로 복귀
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
  // BLDC PWM 초기화
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255);
  currentBldcPwm = 255;
  digitalWrite(BLDC_PWM_PIN, HIGH);
  delay(50);

  // 시리얼 포트 초기화 (USB / UART)
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial1.begin(9600);  // UART 통신
  Serial1.setTimeout(10);
  delay(500);

  // 스텝 모터 Enable 핀
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
  delay(10);

  // 모터1 스텝/방향 핀 초기화
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);

  // 모터2 스텝/방향 핀 초기화
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);

  // BLDC FG 입력 및 인터럽트 설정
  pinMode(BLDC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BLDC_FG_PIN), countFgPulse, RISING);

  // EEPROM에서 모터2 위치 로드
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

  // 사용 가능한 명령 안내
  Serial.println("");
  Serial.println("[ 스테퍼 모터 ]");
  Serial.println("  1 cw/ccw 100/200/...");
  Serial.println("  2 cw/ccw 100/200/...");
  Serial.println("");
  Serial.println("[ BLDC 팬 ]");
  Serial.println("  3 speed 0~5, fan-test 명령 추가됨");
  Serial.println("  (부팅 시 안전 정지)");
  Serial.println("");
  Serial.println("[ UART 제어 ]");
  Serial.println("  A r/l/u/d 1/0 (각도 제어)");
  Serial.println("  S 0~5 (BLDC 속도)");
  Serial.println("");
  Serial.println("명령: home / sethome / clear / fan-test");
  Serial.println("Ready!");

  lastRpmTime = millis();
}

// 모터2 위치를 EEPROM에 저장
void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  EEPROM.put(0, dataToSave);
  EEPROM.commit();
  delay(10);
}

// EEPROM 데이터 제거 및 홈 위치로 초기화
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

// 가·감속 프로파일을 포함한 모터 스텝 실행
void stepMotor(int motorNum, int steps, int initialDelay) {
  int stepPin, dirPin, enablePin;

  // 모터 번호에 따라 핀 선택
  if (motorNum == 1) {
    stepPin = STEP_PIN1;
    dirPin = DIR_PIN1;
    enablePin = ENABLE_PIN1;
  } else {
    stepPin = STEP_PIN2;
    dirPin = DIR_PIN2;
    enablePin = ENABLE_PIN2;
  }

  // 모터 Enable
  digitalWrite(enablePin, LOW);
  delayMicroseconds(1000);

  // 가·감속에 사용되는 딜레이 파라미터
  const int START_SPEED = 1500;   // 시작 시 느린 속도(딜레이)
  const int MAX_SPEED = 500;      // 최고 속도(최소 딜레이)
  const int SPEED_STEP = 100;     // 딜레이 계단 단위

  // 전체 스텝에서 가속/감속 구간 비율 계산
  int accelSteps = (steps * 3) / 10;
  if (accelSteps < 80) accelSteps = 80;
  if (accelSteps > 400) accelSteps = 400;

  int decelSteps = accelSteps;
  int constantSteps = steps - accelSteps - decelSteps;

  // 스텝 수가 적을 때 가·감속만 사용
  if (constantSteps < 0) {
    accelSteps = steps / 2;
    decelSteps = steps - accelSteps;
    constantSteps = 0;
  }

  // 가속 → 등속 → 감속 프로파일로 스텝 실행
  for (int i = 0; i < steps; i++) {
    int currentDelay;

    if (i < accelSteps) {
      // 가속 구간 (스무딩된 이징 함수)
      float ratio = (float)i / accelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = START_SPEED - (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    } else if (i < accelSteps + constantSteps) {
      // 일정 속도 구간
      currentDelay = MAX_SPEED;
    } else {
      // 감속 구간
      int decelProgress = i - accelSteps - constantSteps;
      float ratio = (float)decelProgress / decelSteps;
      ratio = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);
      currentDelay = MAX_SPEED + (START_SPEED - MAX_SPEED) * ratio;
      currentDelay = ((currentDelay + 50) / SPEED_STEP) * SPEED_STEP;
    }

    // 실제 스텝 펄스 출력
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
  }

  // 스텝 종료 후 Enable 해제
  delayMicroseconds(500);
  digitalWrite(enablePin, HIGH);
  delay(1);

  // BLDC PWM 설정 복구
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, currentBldcPwm);
}

// BLDC 속도 레벨 설정 (부드러운 램프 업/다운)
void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  int targetPwm = SPEED_LEVELS[level];

  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);

  // 현재 PWM에서 목표 PWM까지 1씩 이동
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

// 1초 주기로 BLDC RPM 출력
void measureRPM() {
  unsigned long currentTime = millis();

  if (currentTime - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3;  // FG 3펄스 = 1회전 가정
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

// UART 명령 처리 (모터1/2 + BLDC)
void processUartCommand() {
  if (Serial1.available() > 0) {
    String input = Serial1.readStringUntil('\n');
    input.trim();

    // 수신한 UART 명령 로그 출력
    Serial.print("[UART] ");
    Serial.println(input);

    // "S n" : BLDC 속도 제어
    if (input.startsWith("S ")) {
      int sLevel = input.substring(2).toInt();
      if (sLevel >= 0 && sLevel <= 5) {
        setBldcSpeed(sLevel);
        Serial.print("UART set BLDC speed = ");
        Serial.println(sLevel);
      }
      return; // S 명령 후에는 다른 명령 무시
    }

    // "A d 1/0" : 각도 제어 (좌/우/상/하)
    if (input.startsWith("A ")) {
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > 0) {
        char direction = input.charAt(firstSpace + 1);
        int state = input.substring(secondSpace + 1).toInt();

        // 모터1 (좌우): r=RIGHT(CW), l=LEFT(CCW)
        if (direction == 'r' || direction == 'l') {
          if (state == 1) {
            motor1_direction = (direction == 'r') ? 1 : 2;
            motor1_moving = true;
            digitalWrite(DIR_PIN1, (direction == 'r') ? HIGH : LOW);
            digitalWrite(ENABLE_PIN1, LOW);
            Serial.print("Motor1 ");
            Serial.print((direction == 'r') ? "RIGHT" : "LEFT");
            Serial.println(" START");
          } else {
            motor1_moving = false;
            motor1_direction = 0;
            digitalWrite(ENABLE_PIN1, HIGH);
            Serial.print("Motor1 STOP (pos: ");
            Serial.print(motor1_position);
            Serial.println(")");
          }
        }

        // 모터2 (상하): u=UP(CCW), d=DOWN(CW)
        if (direction == 'u' || direction == 'd') {
          if (state == 1) {
            motor2_direction_uart = (direction == 'd') ? 1 : 2;
            motor2_moving_uart = true;
            digitalWrite(DIR_PIN2, (direction == 'd') ? HIGH : LOW);
            digitalWrite(ENABLE_PIN2, LOW);
            Serial.print("Motor2 ");
            Serial.print((direction == 'u') ? "UP" : "DOWN");
            Serial.println(" START");
          } else {
            motor2_moving_uart = false;
            motor2_direction_uart = 0;
            digitalWrite(ENABLE_PIN2, HIGH);
            savePositionToFlash();
            Serial.print("Motor2 STOP (pos: ");
            Serial.print(motor2_position);
            Serial.println(")");
          }
        }
      }
    }
  }
}

// UART 기반 모터 구동 (속도 제한 및 리밋 포함)
void driveUartMotors() {
  static unsigned long lastStepTime1 = 0;
  static unsigned long lastStepTime2 = 0;
  unsigned long currentTime = micros();
  static int skipCounter = 0;  // 호출 스킵용 카운터

  // 모터 속도를 줄이기 위해 일정 횟수 스킵
  skipCounter++;
  if (skipCounter < 3) return;  // 3번 중 1번만 실제 실행
  skipCounter = 0;

  // 모터1 구동 (좌우)
  if (motor1_moving && (currentTime - lastStepTime1 >= UART_MOTOR_DELAY * 2)) {
    bool canMove = true;
    // 리밋 체크
    if (motor1_direction == 1 && motor1_position <= MOTOR1_CW_LIMIT) {
      canMove = false;
    }
    if (motor1_direction == 2 && motor1_position >= MOTOR1_CCW_LIMIT) {
      canMove = false;
    }

    if (canMove) {
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(5);  // 최소 펄스 폭
      digitalWrite(STEP_PIN1, LOW);

      if (motor1_direction == 1) motor1_position--;
      else motor1_position++;
    }

    lastStepTime1 = currentTime;
  }

  // 모터2 구동 (상하)
  if (motor2_moving_uart && (currentTime - lastStepTime2 >= UART_MOTOR_DELAY * 2)) {
    bool canMove = true;
    // 리밋 체크
    if (motor2_direction_uart == 1 && motor2_position <= HOME_POSITION) {
      canMove = false;
    }
    if (motor2_direction_uart == 2 && motor2_position >= MAX_CCW_STEPS) {
      canMove = false;
    }

    if (canMove) {
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(5);  // 최소 펄스 폭
      digitalWrite(STEP_PIN2, LOW);

      if (motor2_direction_uart == 1) motor2_position--;
      else motor2_position++;
    }

    lastStepTime2 = currentTime;
  }
}

void loop() {
  // UART 명령 수신 및 반영
  processUartCommand();
  // UART 연속 모터 구동
  driveUartMotors();

  // BLDC RPM 모니터링
  if (bldcSpeedLevel > 0) {
    measureRPM();
  }

  // USB 명령 처리
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // EEPROM 클리어
    if (input == "clear") {
      clearFlashMemory();
      return;
    }

    // 현재 위치를 홈으로 설정
    if (input == "sethome") {
      motor2_position = HOME_POSITION;
      savePositionToFlash();
      Serial.println("✓");
      return;
    }

    // 모터2를 홈 위치로 물리 이동
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

    // BLDC 속도 설정 명령: "3 speed n"
    if (input.startsWith("3 speed ")) {
      int level = input.substring(8).toInt();
      if (level >= 0 && level <= 5) {
        setBldcSpeed(level);
      } else {
        Serial.println("✗ 0~5");
      }
      return;
    }

    // 팬 PWM 튜닝용 테스트
    if (input == "fan-test") {
      fanTest();
      return;
    }

    // 스텝 모터 수동 이동 명령 파싱: "1 cw 200"
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);

    if (firstSpace > 0 && secondSpace > 0) {
      int motorNum = input.substring(0, firstSpace).toInt();
      String direction = input.substring(firstSpace + 1, secondSpace);
      int steps = input.substring(secondSpace + 1).toInt();
      direction.trim();

      // 100단위 스텝만 허용
      if (steps % 100 != 0) {
        Serial.println("✗ 100단위");
        return;
      }

      // 모터 번호/방향/스텝 유효성 체크
      if ((motorNum == 1 || motorNum == 2) &&
          (direction == "cw" || direction == "ccw") &&
          steps > 0) {

        // 모터2는 소프트 리밋 체크
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

        // 방향 설정
        if (motorNum == 1) {
          digitalWrite(DIR_PIN1, (direction == "cw") ? HIGH : LOW);
        } else {
          digitalWrite(DIR_PIN2, (direction == "cw") ? HIGH : LOW);
        }
        delayMicroseconds(10);

        // 가·감속 포함 스텝 실행
        stepMotor(motorNum, steps, 500);

        // 모터2 위치 카운터 업데이트 및 저장
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

  // 메인 루프 간격 조정 (과도한 바쁨 방지)
  delayMicroseconds(100);
}
