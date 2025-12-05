#include <EEPROM.h>

// 그래프 디버그 모드 플래그
bool graph_debug_mode = false;
unsigned long last_graph_print = 0;

// 카메라 해상도
#define CAM_WIDTH 1920
#define CAM_HEIGHT 1080

// 모터 동작 범위
const int MOTOR1_MIN_STEPS = -10000;
const int MOTOR1_MAX_STEPS = 10000;
const int MOTOR2_HOME = 0;
const int MOTOR2_MAX_STEPS = 8500;

// 스텝 딜레이 (속도 제어용)
#define MIN_DELAY_US 50
#define MAX_DELAY_US 800

// 페이스 트래킹 제어 게인
const float Kp = 3.5;
const float Ki = 0.02;
const int INTEGRAL_LIMIT = 500;

// 얼굴 위치 → 모터 이동 비율
const float PAN_GAIN = 3.0;
const float TILT_GAIN = 3.0;

// 추적 데드존 (중앙 근처 무시 구간)
#define TRACKING_DEADZONE 100

// 스텝 모터 전원 차단 지연
#define ENABLE_OFF_DELAY_MS 50

// EEPROM 자동 저장 지연
#define EEPROM_SAVE_DELAY_MS 3000

// 자연풍 레벨 범위 및 변경 주기
#define NATURAL_WIND_MIN_LEVEL 1
#define NATURAL_WIND_MAX_LEVEL 3
#define NATURAL_WIND_CHANGE_INTERVAL_MS 3000

// 회전 모드 방향 전환 대기시간
#define ROTATION_PAUSE_MS 1000

// 핀 정의
#define STEP_PIN2 D4
#define DIR_PIN2 D3
#define ENABLE_PIN2 D5
#define STEP_PIN1 D1
#define DIR_PIN1 D10
#define ENABLE_PIN1 D2
#define BLDC_PWM_PIN D8
#define BLDC_FG_PIN D9

// 모터 위치 저장용 구조체 (EEPROM)
typedef struct {
  int motor2_position; // 틸트 모터 현재 위치
  int valid;           // 유효 데이터 시그니처
} MotorPosition;

// 모터 위치 상태 값
int motor2_position = 0;
int motor1_position = 0;
const int VALID_SIGNATURE = 12345; // EEPROM 데이터 유효 여부 체크용 값

// EEPROM 저장 플래그 및 시간
bool motor2_needs_save = false;
unsigned long last_motor2_move_time = 0;

// BLDC 속도 상태
int bldcSpeedLevel = 0;
int currentBldcPwm = 255;
int targetBldcPwm = 255;
unsigned long lastBldcUpdate = 0;
// 속도 레벨별 PWM 값 테이블
const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

// FG 신호 기반 RPM 측정용 변수
volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

// 수동 조작 상태 플래그
bool motor1_manual_moving = false;
bool motor2_manual_moving = false;
int motor1_manual_dir = 0;
int motor2_manual_dir = 0;

// 테스트 이동 명령용 변수
int test_motor1_steps = 0;
int test_motor1_dir = 0;
int test_motor2_steps = 0;
int test_motor2_dir = 0;

// 페이스 트래킹 상태 및 목표 위치
bool facetracking_enabled = false;
int target_motor1_pos = 0;
int target_motor2_pos = 0;

// 스텝 타이머 (비차단 제어용)
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

// PID 적분 항 (현재는 사용 거의 없음)
float integral_error_1 = 0;
float integral_error_2 = 0;

// 모터 동작 시각 기록
unsigned long last_motor1_action_time = 0;
unsigned long last_motor2_action_time = 0;

// 마지막 스텝 시간 (전원 자동 OFF 기준)
unsigned long last_motor1_step_time = 0;
unsigned long last_motor2_step_time = 0;

// 자동 왕복 회전 모드 상태
bool rotation_mode = false;
int rotation_direction = 1;

// 자연풍 모드 상태
bool natural_wind_mode = false;
unsigned long last_natural_wind_change = 0;
int natural_wind_target_level = 1;
bool natural_wind_increasing = true;

// 회전 모드 일시 정지 상태
unsigned long last_rotation_step = 0;
bool rotation_pausing = false;
unsigned long rotation_pause_start = 0;

// BLDC FG 인터럽트 콜백 (펄스 카운트)
void countFgPulse() { fgPulseCount++; }

void setup() {
  // BLDC 제어 핀 설정
  pinMode(BLDC_PWM_PIN, OUTPUT);
  digitalWrite(BLDC_PWM_PIN, HIGH);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255);

  // 디버그 / 데이터용 직렬 포트
  Serial.begin(921600);
  Serial.setTimeout(2);
  Serial1.begin(921600);
  Serial1.setTimeout(2);

  // 스텝 모터 드라이버 Enable 핀
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH); // 초기에는 비활성화
  digitalWrite(ENABLE_PIN2, HIGH);

  // 스텝 / 방향 핀 설정
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);

  // BLDC FG 입력 및 인터럽트 설정
  pinMode(BLDC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BLDC_FG_PIN), countFgPulse, RISING);

  // EEPROM 초기화 및 모터 위치 로드
  EEPROM.begin(512);
  MotorPosition savedData;
  EEPROM.get(0, savedData);

  if (savedData.valid == VALID_SIGNATURE &&
      savedData.motor2_position >= MOTOR2_HOME &&
      savedData.motor2_position <= MOTOR2_MAX_STEPS) {
    motor2_position = savedData.motor2_position;
    Serial.print("Load Pos: ");
    Serial.println(motor2_position);
  } else {
    motor2_position = MOTOR2_HOME;
    savePositionToFlash();
  }

  Serial.println("Ready (High Speed 1/32 Tracking)");
  lastRpmTime = millis();
}

// 모터2 위치를 EEPROM에 저장
void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  EEPROM.put(0, dataToSave);
  EEPROM.commit();
  Serial.println("Saved.");
}

// EEPROM 초기화 및 홈 위치로 리셋
void clearFlashMemory() {
  MotorPosition emptyData;
  emptyData.motor2_position = MOTOR2_HOME;
  emptyData.valid = 0;
  EEPROM.put(0, emptyData);
  EEPROM.commit();
  motor2_position = MOTOR2_HOME;
  motor2_needs_save = false;
  Serial.println("Cleared.");
}

// 일정 시간 지나면 모터2 위치 자동 저장
void checkAutoSave() {
  if (motor2_needs_save && (millis() - last_motor2_move_time >= EEPROM_SAVE_DELAY_MS)) {
    savePositionToFlash();
    motor2_needs_save = false;
  }
}

// BLDC 속도 레벨 설정
void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  targetBldcPwm = (level == 0) ? 255 : SPEED_LEVELS[level];
  bldcSpeedLevel = level;
}

// BLDC PWM을 서서히 목표 값으로 보정
void updateBldcSpeed() {
  if (millis() - lastBldcUpdate >= 6) {
    if (currentBldcPwm < targetBldcPwm) currentBldcPwm++;
    else if (currentBldcPwm > targetBldcPwm) currentBldcPwm--;
    analogWrite(BLDC_PWM_PIN, currentBldcPwm);
    lastBldcUpdate = millis();
  }
}

// FG 펄스 기반 RPM 계산
void measureRPM() {
  if (millis() - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3; // FG 3펄스당 1회전 가정
      fgPulseCount = 0;
    }
    lastRpmTime = millis();
  }
}

// Serial1(UART) 명령 처리 (페이스 트래킹 / 속도 / 회전 / 수동 조작)
void processUartCommand() {
  if (Serial1.available() > 0) {
    String lastValidCommand = "";
    while (Serial1.available() > 0) {
      String temp = Serial1.readStringUntil('\n');
      temp.trim();
      if (temp.length() > 0) {
        lastValidCommand = temp;
      }
    }

    if (lastValidCommand == "") return;
    String input = lastValidCommand;

    // 얼굴 좌표(P (x,y)) → 목표 스텝으로 변환
    if (input.startsWith("P (")) {
      int openParen = input.indexOf('(');
      int commaIdx = input.indexOf(',');
      int closeParen = input.indexOf(')');
      if (openParen >= 0 && commaIdx > openParen && closeParen > commaIdx) {
        int px = input.substring(openParen + 1, commaIdx).toInt();
        int py = input.substring(commaIdx + 1, closeParen).toInt();
        px = constrain(px, 0, CAM_WIDTH);
        py = constrain(py, 0, CAM_HEIGHT);

        int error_x = px - (CAM_WIDTH / 2);
        int delta_step_1 = error_x * PAN_GAIN;
        target_motor1_pos = motor1_position + delta_step_1;
        target_motor1_pos = constrain(target_motor1_pos, MOTOR1_MIN_STEPS, MOTOR1_MAX_STEPS);

        int error_y = py - (CAM_HEIGHT / 2);
        int delta_step_2 = error_y * TILT_GAIN;
        target_motor2_pos = motor2_position + delta_step_2;
        target_motor2_pos = constrain(target_motor2_pos, MOTOR2_HOME, MOTOR2_MAX_STEPS);

        facetracking_enabled = true;
        rotation_mode = false;
        motor1_manual_moving = false;
        motor2_manual_moving = false;
      }
    }
    // 페이스 트래킹 정지
    else if (input == "P X" || input == "P x") {
      facetracking_enabled = false;
      digitalWrite(ENABLE_PIN1, HIGH);
      digitalWrite(ENABLE_PIN2, HIGH);
      if (motor2_needs_save) {
        savePositionToFlash();
        motor2_needs_save = false;
      }
      integral_error_1 = 0;
      integral_error_2 = 0;
    }
    // 고정 풍량 모드 (S n)
    else if (input.startsWith("S ")) {
      natural_wind_mode = false;
      setBldcSpeed(input.substring(2).toInt());
    }
    // 자연풍 모드 (N 1/0)
    else if (input.startsWith("N ")) {
      natural_wind_mode = (input.substring(2).toInt() == 1);
      if (natural_wind_mode) {
        natural_wind_target_level = NATURAL_WIND_MIN_LEVEL;
        natural_wind_increasing = true;
        last_natural_wind_change = millis();
      } else {
        setBldcSpeed(0);
      }
    }
    // 자동 왕복 회전 모드 (R 1/0)
    else if (input.startsWith("R ")) {
      int rMode = input.substring(2).toInt();
      facetracking_enabled = false;

      if (rMode == 1) {
        rotation_mode = true;
        rotation_direction = 1;
        digitalWrite(DIR_PIN1, HIGH);
        digitalWrite(ENABLE_PIN1, LOW);
        rotation_pausing = false;
      } else {
        rotation_mode = false;
        digitalWrite(ENABLE_PIN1, HIGH);
      }
    }
    // 수동 모터 제어 (A d state)
    else if (input.startsWith("A ")) {
      facetracking_enabled = false;
      rotation_mode = false;

      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      if (firstSpace > 0 && secondSpace > 0) {
        char direction = input.charAt(firstSpace + 1);
        int state = input.substring(secondSpace + 1).toInt();
        // 좌우 조작
        if (direction == 'r' || direction == 'l') {
          if (state == 1) {
            motor1_manual_dir = (direction == 'r') ? 1 : 2;
            motor1_manual_moving = true;
            digitalWrite(DIR_PIN1, (direction == 'r') ? HIGH : LOW);
            digitalWrite(ENABLE_PIN1, LOW);
          } else {
            motor1_manual_moving = false;
            digitalWrite(ENABLE_PIN1, HIGH);
          }
        }
        // 상하 조작
        else if (direction == 'u' || direction == 'd') {
          if (state == 1) {
            motor2_manual_dir = (direction == 'u') ? 1 : 2;
            motor2_manual_moving = true;
            digitalWrite(DIR_PIN2, (direction == 'u') ? HIGH : LOW);
            digitalWrite(ENABLE_PIN2, LOW);
          } else {
            motor2_manual_moving = false;
            digitalWrite(ENABLE_PIN2, HIGH);
            motor2_needs_save = true;
            last_motor2_move_time = millis();
          }
        }
      }
    }
  }
}

// USB Serial 디버그 명령 처리
void processDebugCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // 그래프 출력 토글
    if (input == "graph") {
      graph_debug_mode = !graph_debug_mode;
      if(graph_debug_mode) Serial1.println("Time,Target,Current");
      else Serial1.println("Graph Mode OFF");
      return;
    }

    // EEPROM 초기화
    if (input == "clear") {
      clearFlashMemory();
      return;
    }
    // 현재 위치를 홈으로 설정
    if (input == "sethome") {
      motor2_position = MOTOR2_HOME;
      savePositionToFlash();
      return;
    }
    // 단발 테스트 이동 명령: "1 cw 1000" / "2 ccw 500" 등
    if (input.startsWith("1 ") || input.startsWith("2 ")) {
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.lastIndexOf(' ');
      if (firstSpace > 0 && secondSpace > firstSpace) {
        int motorId = input.substring(0, firstSpace).toInt();
        String dirStr = input.substring(firstSpace + 1, secondSpace);
        int steps = input.substring(secondSpace + 1).toInt();
        int dirVal = (dirStr == "cw") ? 1 : -1;

        facetracking_enabled = false;
        rotation_mode = false;
        motor1_manual_moving = false;
        motor2_manual_moving = false;

        if (motorId == 1) {
          test_motor1_steps = steps;
          test_motor1_dir = dirVal;
        } else if (motorId == 2) {
          test_motor2_steps = steps;
          test_motor2_dir = dirVal;
        }
      }
    }
  }
}

// 테스트 이동 모드에서 지정된 스텝만큼 이동
void driveTestMotors() {
  unsigned long now_ms = millis();

  // 모터1 테스트 이동
  if (test_motor1_steps > 0) {
    bool canMove = true;
    if (test_motor1_dir == 1 && motor1_position >= MOTOR1_MAX_STEPS) canMove = false;
    if (test_motor1_dir == -1 && motor1_position <= MOTOR1_MIN_STEPS) canMove = false;

    if (canMove) {
      digitalWrite(ENABLE_PIN1, LOW);
      digitalWrite(DIR_PIN1, (test_motor1_dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN1, LOW);
      motor1_position += test_motor1_dir;
      last_motor1_step_time = now_ms;
    } else {
      digitalWrite(ENABLE_PIN1, HIGH);
    }

    delayMicroseconds(MIN_DELAY_US);
    test_motor1_steps--;

    if (test_motor1_steps == 0) digitalWrite(ENABLE_PIN1, HIGH);
  }

  // 모터2 테스트 이동
  if (test_motor2_steps > 0) {
    bool canMove = true;
    if (test_motor2_dir == 1 && motor2_position >= MOTOR2_MAX_STEPS) canMove = false;
    if (test_motor2_dir == -1 && motor2_position <= MOTOR2_HOME) canMove = false;

    if (canMove) {
      digitalWrite(ENABLE_PIN2, LOW);
      digitalWrite(DIR_PIN2, (test_motor2_dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);
      motor2_position += test_motor2_dir;
      motor2_needs_save = true;
      last_motor2_move_time = now_ms;
      last_motor2_step_time = now_ms;
    } else {
      digitalWrite(ENABLE_PIN2, HIGH);
    }

    delayMicroseconds(MIN_DELAY_US);
    test_motor2_steps--;

    if (test_motor2_steps == 0) digitalWrite(ENABLE_PIN2, HIGH);
  }
}

// 수동 조작 모드에서 연속 스텝 구동
void driveManualMotors() {
  const long MANUAL_STEP_DELAY = 50; // 수동 모드 스텝 간 딜레이
  unsigned long now_ms = millis();

  // 모터1 수동 이동
  if (motor1_manual_moving) {
    bool canMove = true;
    if (motor1_manual_dir == 1 && motor1_position <= MOTOR1_MIN_STEPS) canMove = false;
    if (motor1_manual_dir == 2 && motor1_position >= MOTOR1_MAX_STEPS) canMove = false;

    if (canMove) {
      digitalWrite(ENABLE_PIN1, LOW);
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN1, LOW);
      motor1_position += (motor1_manual_dir == 1) ? -1 : 1;
      last_motor1_step_time = now_ms;
      delayMicroseconds(MANUAL_STEP_DELAY);
    } else {
      digitalWrite(ENABLE_PIN1, HIGH);
    }
  }

  // 모터2 수동 이동
  if (motor2_manual_moving) {
    bool canMove = true;
    if (motor2_manual_dir == 1 && motor2_position <= MOTOR2_HOME) canMove = false;
    if (motor2_manual_dir == 2 && motor2_position >= MOTOR2_MAX_STEPS) canMove = false;

    if (canMove) {
      digitalWrite(ENABLE_PIN2, LOW);
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);
      motor2_position += (motor2_manual_dir == 1) ? -1 : 1;
      motor2_needs_save = true;
      last_motor2_move_time = millis();
      last_motor2_step_time = now_ms;
      delayMicroseconds(MANUAL_STEP_DELAY);
    } else {
      digitalWrite(ENABLE_PIN2, HIGH);
    }
  }
}

// 자동 왕복 회전 모드 제어
void rotationMode() {
  if (!rotation_mode) return;

  // 회전 방향 전환을 위한 일시 정지 구간
  if (rotation_pausing) {
    if (millis() - rotation_pause_start >= ROTATION_PAUSE_MS) {
      rotation_pausing = false;
      rotation_direction = (rotation_direction == 1) ? 2 : 1;
      digitalWrite(DIR_PIN1, (rotation_direction == 1) ? HIGH : LOW);
      digitalWrite(ENABLE_PIN1, LOW);
    } else {
      digitalWrite(ENABLE_PIN1, HIGH);
      return;
    }
  }

  // 범위 초과 시 방향 전환 준비
  bool hit_limit = false;
  if (rotation_direction == 1 && motor1_position <= MOTOR1_MIN_STEPS) hit_limit = true;
  if (rotation_direction == 2 && motor1_position >= MOTOR1_MAX_STEPS) hit_limit = true;

  if (hit_limit) {
    rotation_pausing = true;
    rotation_pause_start = millis();
    return;
  }

  // 일정 속도로 연속 스텝 (부드러운 회전)
  digitalWrite(ENABLE_PIN1, LOW);
  digitalWrite(STEP_PIN1, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN1, LOW);

  motor1_position += (rotation_direction == 1) ? -1 : 1;
  last_motor1_step_time = millis();

  delayMicroseconds(60);
}

// 자연풍 모드에서 속도 레벨을 주기적으로 변경
void naturalWindMode() {
  if (!natural_wind_mode) return;
  if (millis() - last_natural_wind_change >= NATURAL_WIND_CHANGE_INTERVAL_MS) {
    if (natural_wind_increasing) {
      if (++natural_wind_target_level >= NATURAL_WIND_MAX_LEVEL) natural_wind_increasing = false;
    } else {
      if (--natural_wind_target_level <= NATURAL_WIND_MIN_LEVEL) natural_wind_increasing = true;
    }
    setBldcSpeed(natural_wind_target_level);
    last_natural_wind_change = millis();
  }
}

// 페이스 트래킹용 스텝 모터 속도/위치 제어 (P제어 + 속도 스무딩)
void faceTrackingControl() {
  if (!facetracking_enabled) return;

  unsigned long now = micros();
  unsigned long now_ms = millis();

  // 각 모터 속도 스무딩용 지연 값
  static float smooth_delay_1 = MAX_DELAY_US;
  static float smooth_delay_2 = MAX_DELAY_US;

  // 실제 적용 중인 딜레이 (속도 고정 구간)
  static long applied_delay_1 = MAX_DELAY_US;
  static long applied_delay_2 = MAX_DELAY_US;

  // 속도 변경 히스테리시스 (주파수 안정화 용도)
  const int SPEED_HYSTERESIS = 20;
  const float ACCEL_FACTOR = 0.05;

  // ===== 모터1 (Pan) =====
  int error1 = target_motor1_pos - motor1_position;

  bool blocked1 = false;
  if (error1 > 0 && motor1_position >= MOTOR1_MAX_STEPS) blocked1 = true;
  if (error1 < 0 && motor1_position <= MOTOR1_MIN_STEPS) blocked1 = true;

  if (abs(error1) <= TRACKING_DEADZONE || blocked1) {
    if (blocked1 || (now_ms - last_motor1_action_time > ENABLE_OFF_DELAY_MS)) {
      digitalWrite(ENABLE_PIN1, HIGH);
    }
    smooth_delay_1 = MAX_DELAY_US;
  }
  else {
    float control_signal = (3.0 * error1);
    long target_delay = map((long)abs(control_signal), 0, 1000, 1500, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, 1500);

    smooth_delay_1 = (smooth_delay_1 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);

    if (abs(smooth_delay_1 - applied_delay_1) > SPEED_HYSTERESIS) {
      applied_delay_1 = (long)smooth_delay_1;
    }

    if (now - lastStepTime1 >= applied_delay_1) {
      digitalWrite(ENABLE_PIN1, LOW);
      int dir = (error1 > 0) ? 1 : -1;

      digitalWrite(DIR_PIN1, (dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN1, LOW);

      motor1_position += dir;
      last_motor1_action_time = now_ms;
      last_motor1_step_time = now_ms;
      lastStepTime1 = now;
    }
  }

  // ===== 모터2 (Tilt) =====
  int error2 = target_motor2_pos - motor2_position;

  bool blocked2 = false;
  if (error2 > 0 && motor2_position >= MOTOR2_MAX_STEPS) blocked2 = true;
  if (error2 < 0 && motor2_position <= MOTOR2_HOME) blocked2 = true;

  if (abs(error2) <= TRACKING_DEADZONE || blocked2) {
    if (blocked2 || (now_ms - last_motor2_action_time > ENABLE_OFF_DELAY_MS)) {
      digitalWrite(ENABLE_PIN2, HIGH);
    }
    smooth_delay_2 = MAX_DELAY_US;
  }
  else {
    float control_signal = (3.0 * error2);
    long target_delay = map((long)abs(control_signal), 0, 1000, 1200, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, 1200);

    smooth_delay_2 = (smooth_delay_2 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);

    if (abs(smooth_delay_2 - applied_delay_2) > SPEED_HYSTERESIS) {
      applied_delay_2 = (long)smooth_delay_2;
    }

    if (now - lastStepTime2 >= applied_delay_2) {
      digitalWrite(ENABLE_PIN2, LOW);
      int dir = (error2 > 0) ? 1 : -1;

      digitalWrite(DIR_PIN2, (dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);

      motor2_position += dir;
      motor2_needs_save = true;
      last_motor2_move_time = now_ms;
      last_motor2_action_time = now_ms;
      last_motor2_step_time = now_ms;
      lastStepTime2 = now;
    }
  }

  // 디버그 그래프 출력 (시간, 목표 위치, 현재 위치)
  if (graph_debug_mode && (now_ms - last_graph_print >= 10)) {
    Serial1.print(now_ms);
    Serial1.print(",");
    Serial1.print(target_motor1_pos);
    Serial1.print(",");
    Serial1.println(motor1_position);
    last_graph_print = now_ms;
  }
}

// 일정 시간 이상 움직임 없으면 스텝 모터 전원 차단
void checkMotorPowerOff() {
  unsigned long now_ms = millis();

  if (!motor1_manual_moving && !rotation_mode && !facetracking_enabled && test_motor1_steps == 0) {
    if (now_ms - last_motor1_step_time >= ENABLE_OFF_DELAY_MS) {
      digitalWrite(ENABLE_PIN1, HIGH);
    }
  }

  if (!motor2_manual_moving && !facetracking_enabled && test_motor2_steps == 0) {
    if (now_ms - last_motor2_step_time >= ENABLE_OFF_DELAY_MS) {
      digitalWrite(ENABLE_PIN2, HIGH);
    }
  }
}

void loop() {
  // 명령 수신 처리
  processUartCommand();
  processDebugCommand();

  // 모드 우선순위에 따른 모터 제어
  if (motor1_manual_moving || motor2_manual_moving) driveManualMotors();
  else if (test_motor1_steps > 0 || test_motor2_steps > 0) driveTestMotors();
  else if (rotation_mode) rotationMode();
  else if (facetracking_enabled) faceTrackingControl();

  // 전원 관리 및 부가 기능
  checkMotorPowerOff();
  naturalWindMode();
  updateBldcSpeed();
  checkAutoSave();

  // BLDC RPM 측정
  if (bldcSpeedLevel > 0) measureRPM();
}
