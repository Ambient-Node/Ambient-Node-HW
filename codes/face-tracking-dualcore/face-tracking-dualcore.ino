#include <EEPROM.h>

// 카메라 해상도
#define CAM_WIDTH 1920
#define CAM_HEIGHT 1080

// 모터 동작 범위
const int MOTOR1_MIN_STEPS = -10000;
const int MOTOR1_MAX_STEPS = 10000;
const int MOTOR2_HOME = 0;
const int MOTOR2_MAX_STEPS = 8500;

// 속도 설정 (Core 1: 스텝 딜레이 범위)
#define MIN_DELAY_US 50            
#define MAX_DELAY_US 600           

// PID 제어 게인 (페이스 트래킹용)
const float Kp = 4.0;              
const float Ki = 0.05;             
const int INTEGRAL_LIMIT = 800;    

// 얼굴 좌표 → 모터 스텝 변환 비율
const float PAN_GAIN = 3.2;        
const float TILT_GAIN = 3.2;       

// 트래킹 데드존 (중앙 근처 무시 구간)
#define TRACKING_DEADZONE 15       

// 스텝 모터 전원 차단 지연
#define ENABLE_OFF_DELAY_MS 50     

// EEPROM 자동 저장 지연
#define EEPROM_SAVE_DELAY_MS 3000

// 자연풍 모드 레벨 범위 및 변경 주기
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

// EEPROM에 저장할 모터 위치 구조체
typedef struct {
  int motor2_position;  // 틸트 모터 위치
  int valid;            // 데이터 유효 플래그
} MotorPosition;

// 모터 현재 위치 (Core 1에서 갱신, Core 0에서 참조)
volatile int motor2_position = 0;
volatile int motor1_position = 0;

// 목표 위치 (Core 0에서 설정, Core 1에서 추종)
volatile int target_motor1_pos = 0;
volatile int target_motor2_pos = 0;

// 동작 모드 플래그 (양 코어 공유)
volatile bool facetracking_enabled = false;
volatile bool rotation_mode = false;
volatile int rotation_direction = 1;

// 수동 제어 상태 (양 코어 공유)
volatile bool motor1_manual_moving = false;
volatile bool motor2_manual_moving = false;
volatile int motor1_manual_dir = 0;
volatile int motor2_manual_dir = 0;

// 테스트 이동 상태 (양 코어 공유)
volatile int test_motor1_steps = 0;
volatile int test_motor1_dir = 0;
volatile int test_motor2_steps = 0;
volatile int test_motor2_dir = 0;

// EEPROM 저장 관련 (Core 0 전담)
bool motor2_needs_save = false;
unsigned long last_motor2_move_time = 0;
const int VALID_SIGNATURE = 12345;

// BLDC 속도 제어 및 상태 (Core 0 전담)
int bldcSpeedLevel = 0;
int currentBldcPwm = 255;
int targetBldcPwm = 255;
unsigned long lastBldcUpdate = 0;
const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

// BLDC RPM 측정을 위한 FG 카운터
volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

// 자연풍 모드 상태 (Core 0 전담)
bool natural_wind_mode = false;
unsigned long last_natural_wind_change = 0;
int natural_wind_target_level = 1;
bool natural_wind_increasing = true;

// 모터 상태 변수 (Core 1 전용: 타이밍·PID 상태)
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;
float integral_error_1 = 0;
float integral_error_2 = 0;
unsigned long last_motor1_action_time = 0;
unsigned long last_motor2_action_time = 0;
unsigned long last_motor1_step_time = 0;
unsigned long last_motor2_step_time = 0;

// 회전 모드 상태 (Core 1 전용)
unsigned long last_rotation_step = 0;
bool rotation_pausing = false;
unsigned long rotation_pause_start = 0;

// 디버그 플래그
bool graph_debug_mode = false;
unsigned long last_graph_print = 0;

// BLDC FG 인터럽트 콜백 (펄스 카운트 증가)
void countFgPulse() { fgPulseCount++; }

// =========================================================
// Core 0: 통신, 명령 파싱, BLDC, EEPROM, 자연풍 관리
// =========================================================
void setup() {
  // BLDC 기본 설정 (PWM 주파수 및 초기 상태)
  pinMode(BLDC_PWM_PIN, OUTPUT);
  digitalWrite(BLDC_PWM_PIN, HIGH);  
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255);  

  // 디버그 및 명령 수신용 시리얼
  Serial.begin(921600);
  Serial.setTimeout(2); 
  
  Serial1.begin(921600);
  Serial1.setTimeout(2);

  // 스텝 모터 Enable 핀 및 기본 상태
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH);  
  digitalWrite(ENABLE_PIN2, HIGH);  
  
  // 스텝·방향 핀 초기화
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

  // EEPROM 초기화 및 모터2 위치 로드
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

  // 목표 위치 초기값을 현재 위치로 동기화
  target_motor1_pos = motor1_position;
  target_motor2_pos = motor2_position;

  Serial.println("Ready (Dual Core Architecture)");
  lastRpmTime = millis();
}

void loop() {
  // Core 0: 통신·상위 로직/상태 업데이트 전담
  processUartCommand();
  processDebugCommand();
  
  naturalWindMode();
  updateBldcSpeed();
  checkAutoSave();

  if (bldcSpeedLevel > 0) measureRPM();
}

// =========================================================
// Core 1: 모터 구동 전담 (실시간 제어)
// =========================================================
void setup1() {
  // Core 0의 초기화가 끝나도록 약간 대기
  delay(100); 
}

void loop1() {
  // Core 1: 현재 모드에 따른 모터 제어 실행
  if (motor1_manual_moving || motor2_manual_moving) {
    driveManualMotors();
  }
  else if (test_motor1_steps > 0 || test_motor2_steps > 0) {
    driveTestMotors();
  }
  else if (rotation_mode) {
    rotationMode();
  }
  else if (facetracking_enabled) {
    faceTrackingControl();
  }
  
  // 일정 시간 정지 시 모터 전원 OFF
  checkMotorPowerOff();
}

// =========================================================
// 공용 기능 함수들 (Core 0 중심)
// =========================================================

// 모터2 위치를 EEPROM에 저장
void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  EEPROM.put(0, dataToSave);
  EEPROM.commit();
  Serial.println("Saved.");
}

// EEPROM 초기화 및 모터2 홈 위치로 리셋
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

// 일정 시간 경과 시 모터2 위치 EEPROM 자동 저장
void checkAutoSave() {
  if (motor2_needs_save && (millis() - last_motor2_move_time >= EEPROM_SAVE_DELAY_MS)) {
    savePositionToFlash();
    motor2_needs_save = false;
  }
}

// BLDC 속도 레벨 설정 (0~5)
void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  targetBldcPwm = (level == 0) ? 255 : SPEED_LEVELS[level];
  bldcSpeedLevel = level;
}

// BLDC PWM을 목표값으로 점진적으로 변경
void updateBldcSpeed() {
  if (millis() - lastBldcUpdate >= 6) {
    if (currentBldcPwm < targetBldcPwm) currentBldcPwm++;
    else if (currentBldcPwm > targetBldcPwm) currentBldcPwm--;
    analogWrite(BLDC_PWM_PIN, currentBldcPwm);
    lastBldcUpdate = millis();
  }
}

// FG 펄스로 BLDC RPM을 1초마다 계산
void measureRPM() {
  if (millis() - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3; // FG 3펄스 = 1회전 가정
      fgPulseCount = 0;
    }
    lastRpmTime = millis();
  }
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

// Core 0: UART 명령 파싱 및 공유 변수 업데이트
void processUartCommand() {
  if (Serial1.available() > 0) {
    String lastValidCommand = "";
    // 버퍼를 비우며 마지막 유효 명령만 사용
    while (Serial1.available() > 0) {
      String temp = Serial1.readStringUntil('\n');
      temp.trim();
      if (temp.length() > 0) {
        lastValidCommand = temp;
      }
    }

    if (lastValidCommand == "") return;
    String input = lastValidCommand;

    // P (x,y) : 얼굴 좌표 입력 → 목표 스텝 설정
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
    // P X : 페이스 트래킹 정지
    else if (input == "P X" || input == "P x") {
      facetracking_enabled = false;
      // Enable 핀 OFF는 Core 1에서 checkMotorPowerOff로 처리
      if (motor2_needs_save) {
        savePositionToFlash();
        motor2_needs_save = false;
      }
      integral_error_1 = 0;
      integral_error_2 = 0;
    }
    // S n : 고정 풍량 모드
    else if (input.startsWith("S ")) {
      natural_wind_mode = false;
      setBldcSpeed(input.substring(2).toInt());
    }
    // N 1/0 : 자연풍 모드 ON/OFF
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
    // R 1/0 : 자동 회전 모드 ON/OFF
    else if (input.startsWith("R ")) {
      int rMode = input.substring(2).toInt(); 
      facetracking_enabled = false;
      
      if (rMode == 1) {
        rotation_mode = true;
        rotation_direction = 1;
        digitalWrite(DIR_PIN1, HIGH); // 초기 회전 방향 지정
        rotation_pausing = false;
      } else {
        rotation_mode = false;
        // Enable ON/OFF는 Core 1에서 처리
      }
    }
    // A d state : 수동 제어 명령
    else if (input.startsWith("A ")) {
      facetracking_enabled = false;
      rotation_mode = false;
      
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      if (firstSpace > 0 && secondSpace > 0) {
        char direction = input.charAt(firstSpace + 1);
        int state = input.substring(secondSpace + 1).toInt();
        // 좌우(Pan) 수동 제어
        if (direction == 'r' || direction == 'l') {
          if (state == 1) {
            motor1_manual_dir = (direction == 'r') ? 1 : 2;
            motor1_manual_moving = true;
            digitalWrite(DIR_PIN1, (direction == 'r') ? HIGH : LOW);
          } else {
            motor1_manual_moving = false;
          }
        }
        // 상하(Tilt) 수동 제어
        else if (direction == 'u' || direction == 'd') {
          if (state == 1) {
            motor2_manual_dir = (direction == 'u') ? 1 : 2;
            motor2_manual_moving = true;
            digitalWrite(DIR_PIN2, (direction == 'u') ? HIGH : LOW);
          } else {
            motor2_manual_moving = false;
            motor2_needs_save = true;
            last_motor2_move_time = millis();
          }
        }
      }
    }
  }
}

// Core 0: USB Serial 디버그 명령 처리
void processDebugCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // 그래프 디버그 모드 토글
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
      target_motor2_pos = MOTOR2_HOME;
      savePositionToFlash();
      return;
    }
    // 단발 테스트 이동 명령: "1 cw 1000", "2 ccw 500" 등
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

// -------------------------------------------------------------------------
// Core 1: 모터 구동 로직
// -------------------------------------------------------------------------

// 테스트 모드: 지정 스텝 수만큼 이동
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

// 수동 제어 모드에서 연속 스텝 구동
void driveManualMotors() {
  const long MANUAL_STEP_DELAY = 50; 
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
  unsigned long now_ms = millis();

  // 범위 도달 여부 확인
  bool hit_limit = false;
  if (rotation_direction == 1 && motor1_position <= MOTOR1_MIN_STEPS) hit_limit = true;
  if (rotation_direction == 2 && motor1_position >= MOTOR1_MAX_STEPS) hit_limit = true;

  if (hit_limit) {
    digitalWrite(ENABLE_PIN1, HIGH); // 대기 중 모터 OFF
    
    // Non-blocking 대기: 일정 시간 후 방향 전환
    if (!rotation_pausing) {
      rotation_pausing = true;
      rotation_pause_start = now_ms;
    }
    
    if (now_ms - rotation_pause_start >= ROTATION_PAUSE_MS) {
      rotation_pausing = false;
      rotation_direction = (rotation_direction == 1) ? 2 : 1;
      digitalWrite(DIR_PIN1, (rotation_direction == 1) ? HIGH : LOW);
    }
    return;
  }

  // 일정 속도로 연속 스텝 발생
  digitalWrite(ENABLE_PIN1, LOW);
  digitalWrite(STEP_PIN1, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN1, LOW);
  motor1_position += (rotation_direction == 1) ? -1 : 1;
  last_motor1_step_time = now_ms;
  delayMicroseconds(60); 
}

// 페이스 트래킹: PID+속도 스무딩을 이용한 스텝 제어
void faceTrackingControl() {
  if (!facetracking_enabled) return;

  unsigned long now = micros();
  unsigned long now_ms = millis();

  static float smooth_delay_1 = MAX_DELAY_US;
  static float smooth_delay_2 = MAX_DELAY_US;
  const float ACCEL_FACTOR = 0.2; 

  // Motor 1 (Pan)
  int error1 = target_motor1_pos - motor1_position;
  bool blocked1 = false;
  if (error1 > 0 && motor1_position >= MOTOR1_MAX_STEPS) blocked1 = true;
  if (error1 < 0 && motor1_position <= MOTOR1_MIN_STEPS) blocked1 = true;

  if (abs(error1) <= TRACKING_DEADZONE || blocked1) {
    // 리밋 또는 데드존: 적분 서서히 줄이고 속도 완만히 감소
    if (integral_error_1 > 0) integral_error_1 -= 1;
    else if (integral_error_1 < 0) integral_error_1 += 1;
    
    if (blocked1) {
      digitalWrite(ENABLE_PIN1, HIGH); // 리밋 시 즉시 OFF
    } else if (now_ms - last_motor1_action_time > ENABLE_OFF_DELAY_MS) {
      digitalWrite(ENABLE_PIN1, HIGH);
    }
    smooth_delay_1 = (smooth_delay_1 * 0.98) + (MAX_DELAY_US * 0.02);

  } else {
    // PID 제어 (P + I)
    integral_error_1 += error1;
    integral_error_1 = constrain(integral_error_1, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float control_signal = (Kp * error1) + (Ki * integral_error_1);
    long target_delay = map((long)abs(control_signal), 0, 1000, MAX_DELAY_US, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, MAX_DELAY_US);
    smooth_delay_1 = (smooth_delay_1 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);
    
    if (now - lastStepTime1 >= (long)smooth_delay_1) {
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

  // Motor 2 (Tilt)
  int error2 = target_motor2_pos - motor2_position;
  bool blocked2 = false;
  if (error2 > 0 && motor2_position >= MOTOR2_MAX_STEPS) blocked2 = true;
  if (error2 < 0 && motor2_position <= MOTOR2_HOME) blocked2 = true;

  if (abs(error2) <= TRACKING_DEADZONE || blocked2) {
    if (blocked2) {
      digitalWrite(ENABLE_PIN2, HIGH);
    } else if (now_ms - last_motor2_action_time > ENABLE_OFF_DELAY_MS) {
      digitalWrite(ENABLE_PIN2, HIGH);
    }
    
    smooth_delay_2 = (smooth_delay_2 * 0.98) + (MAX_DELAY_US * 0.02);
    
    if (integral_error_2 > 0) integral_error_2 -= 1;
    else if (integral_error_2 < 0) integral_error_2 += 1;

  } else {
    integral_error_2 += error2;
    integral_error_2 = constrain(integral_error_2, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float control_signal = (Kp * error2) + (Ki * integral_error_2);
    long target_delay = map((long)abs(control_signal), 0, 1000, MAX_DELAY_US, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, MAX_DELAY_US);
    smooth_delay_2 = (smooth_delay_2 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);

    if (now - lastStepTime2 >= (long)smooth_delay_2) {
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

// Core 1: 모터 자동 전원 OFF 관리
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
