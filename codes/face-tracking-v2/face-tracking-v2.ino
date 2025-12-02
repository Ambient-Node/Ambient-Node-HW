#include <EEPROM.h>

// 그래프 디버그 출력 플래그
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

// 스텝 딜레이 범위 (속도 제어)
#define MIN_DELAY_US 50            
#define MAX_DELAY_US 600           

// 페이스 트래킹 PID 계수
const float Kp = 2.5;              
const float Ki = 0.12;             
const int INTEGRAL_LIMIT = 1200;   

// 얼굴 좌표 → 모터 이동 비율
const float PAN_GAIN = 3.2;        
const float TILT_GAIN = 3.2;       

// 트래킹 데드존 (중앙 근처 무시)
#define TRACKING_DEADZONE 20       

// 모드 전체가 유휴 상태일 때 전원 차단까지의 시간
#define IDLE_POWER_OFF_MS 500      

// 기타 설정
#define EEPROM_SAVE_DELAY_MS 3000
#define NATURAL_WIND_MIN_LEVEL 1
#define NATURAL_WIND_MAX_LEVEL 3
#define NATURAL_WIND_CHANGE_INTERVAL_MS 3000

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
  int valid;            // 유효 데이터 플래그
} MotorPosition;

// 모터 현재 위치
int motor2_position = 0;
int motor1_position = 0;
const int VALID_SIGNATURE = 12345; // EEPROM 데이터 유효성 확인용 값

// 모터2 위치 저장 필요 여부
bool motor2_needs_save = false;
unsigned long last_motor2_move_time = 0;

// BLDC 속도 상태
int bldcSpeedLevel = 0;
int currentBldcPwm = 255;
int targetBldcPwm = 255;
unsigned long lastBldcUpdate = 0;
// 속도 레벨별 PWM 값 테이블
const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

// BLDC RPM 측정을 위한 FG 카운터
volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

// 수동 모터 제어 상태
bool motor1_manual_moving = false;
bool motor2_manual_moving = false;
int motor1_manual_dir = 0;
int motor2_manual_dir = 0;

// 테스트 이동 명령 상태
int test_motor1_steps = 0;
int test_motor1_dir = 0;
int test_motor2_steps = 0;
int test_motor2_dir = 0;

// 페이스 트래킹 상태 및 목표 위치
bool facetracking_enabled = false;
int target_motor1_pos = 0;
int target_motor2_pos = 0;

// 비차단 제어용 스텝 타임스탬프
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

// PID 적분 항
float integral_error_1 = 0;
float integral_error_2 = 0;

// 모드 활동 시각 (전원 관리 기준)
unsigned long last_any_mode_active_time = 0;

// 자동 회전 모드 상태
bool rotation_mode = false;
int rotation_direction = 1;

// 자연풍 모드 상태
bool natural_wind_mode = false;
unsigned long last_natural_wind_change = 0;
int natural_wind_target_level = 1;
bool natural_wind_increasing = true;

// 각 모터의 Enable 상태 추적
bool motor1_enabled = false;
bool motor2_enabled = false;

// BLDC FG 인터럽트 콜백 (펄스 카운트 증가)
void countFgPulse() { fgPulseCount++; }

void setup() {
  // BLDC PWM 초기 설정
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255); 

  // 직렬 통신 설정 (USB 디버그 / UART 명령)
  Serial.begin(921600);
  Serial.setTimeout(2); 
  
  Serial1.begin(921600);
  Serial1.setTimeout(2);

  // 스텝 모터 Enable 핀 및 초기 상태
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);

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

  // EEPROM에서 틸트 위치 로드
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

  Serial.println("Ready (Smooth 1/32)");
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

// EEPROM 클리어 및 홈 위치로 리셋
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

// 일정 시간 후 모터2 위치 자동 저장
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

// BLDC PWM 값을 목표 값으로 점진적으로 보정
void updateBldcSpeed() {
  if (millis() - lastBldcUpdate >= 6) {
    if (currentBldcPwm < targetBldcPwm) currentBldcPwm++;
    else if (currentBldcPwm > targetBldcPwm) currentBldcPwm--;
    analogWrite(BLDC_PWM_PIN, currentBldcPwm);
    lastBldcUpdate = millis();
  }
}

// FG 펄스를 이용한 BLDC RPM 측정
void measureRPM() {
  if (millis() - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3; // FG 3펄스 = 1회전 가정
      fgPulseCount = 0;
    }
    lastRpmTime = millis();
  }
}

// 모터1 Enable 제어 헬퍼
void enableMotor1() {
  if (!motor1_enabled) {
    digitalWrite(ENABLE_PIN1, LOW);
    motor1_enabled = true;
    delayMicroseconds(10); // 드라이버 안정화 대기
  }
}

// 모터1 Disable 제어 헬퍼
void disableMotor1() {
  if (motor1_enabled) {
    digitalWrite(ENABLE_PIN1, HIGH);
    motor1_enabled = false;
  }
}

// 모터2 Enable 제어 헬퍼
void enableMotor2() {
  if (!motor2_enabled) {
    digitalWrite(ENABLE_PIN2, LOW);
    motor2_enabled = true;
    delayMicroseconds(10);
  }
}

// 모터2 Disable 제어 헬퍼
void disableMotor2() {
  if (motor2_enabled) {
    digitalWrite(ENABLE_PIN2, HIGH);
    motor2_enabled = false;
  }
}

// UART(Serial1)로 들어오는 명령 처리
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

    // P (x,y) : 얼굴 좌표 기반 목표 위치 설정
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
        
        // 페이스 트래킹 시작 시 모터 활성화
        enableMotor1();
        enableMotor2();
      }
    }
    // P X : 페이스 트래킹 종료
    else if (input == "P X" || input == "P x") {
      facetracking_enabled = false;
      disableMotor1();
      disableMotor2();
      if (motor2_needs_save) {
        savePositionToFlash();
        motor2_needs_save = false;
      }
      integral_error_1 = 0;
      integral_error_2 = 0;
    }
    // S n : 고정 풍량 설정
    else if (input.startsWith("S ")) {
      natural_wind_mode = false;
      setBldcSpeed(input.substring(2).toInt());
    }
    // N 1/0 : 자연풍 모드 On/Off
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
    // R 1/0 : 자동 회전 모드
    else if (input.startsWith("R ")) {
      int rMode = input.substring(2).toInt(); 
      facetracking_enabled = false;
      
      if (rMode == 1) {
        rotation_mode = true;
        rotation_direction = 1;
        digitalWrite(DIR_PIN1, HIGH);
        enableMotor1();
      } else {
        rotation_mode = false;
        disableMotor1();
      }
    }
    // A d state : 수동 조작
    else if (input.startsWith("A ")) {
      facetracking_enabled = false;
      rotation_mode = false;
      
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      if (firstSpace > 0 && secondSpace > 0) {
        char direction = input.charAt(firstSpace + 1);
        int state = input.substring(secondSpace + 1).toInt();
        // 좌우(Pan) 수동
        if (direction == 'r' || direction == 'l') {
          if (state == 1) {
            motor1_manual_dir = (direction == 'r') ? 1 : 2;
            motor1_manual_moving = true;
            digitalWrite(DIR_PIN1, (direction == 'r') ? HIGH : LOW);
            enableMotor1();
          } else {
            motor1_manual_moving = false;
            disableMotor1();
          }
        }
        // 상하(Tilt) 수동
        else if (direction == 'u' || direction == 'd') {
          if (state == 1) {
            motor2_manual_dir = (direction == 'u') ? 1 : 2;
            motor2_manual_moving = true;
            digitalWrite(DIR_PIN2, (direction == 'u') ? HIGH : LOW);
            enableMotor2();
          } else {
            motor2_manual_moving = false;
            disableMotor2();
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

    // 그래프 디버그 모드 토글
    if (input == "graph") {
      graph_debug_mode = !graph_debug_mode;
      if(graph_debug_mode) Serial1.println("Time,Target,Current");
      else Serial1.println("Graph Mode OFF");
      return;
    }

    // EEPROM 클리어
    if (input == "clear") {
      clearFlashMemory();
      return;
    }
    // 홈 위치 설정
    if (input == "sethome") {
      motor2_position = MOTOR2_HOME;
      savePositionToFlash();
      return;
    }
    // 단발 테스트 이동 명령: "1 cw 1000" 등
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

// 테스트 이동 모드에서 지정 스텝만큼 이동
void driveTestMotors() {
  if (test_motor1_steps > 0) {
    bool canMove = true;
    if (test_motor1_dir == 1 && motor1_position >= MOTOR1_MAX_STEPS) canMove = false;
    if (test_motor1_dir == -1 && motor1_position <= MOTOR1_MIN_STEPS) canMove = false;

    if (canMove) {
      enableMotor1();
      digitalWrite(DIR_PIN1, (test_motor1_dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN1, LOW);
      motor1_position += test_motor1_dir;
      last_any_mode_active_time = millis();
    }
    
    delayMicroseconds(MIN_DELAY_US);
    test_motor1_steps--;
    
    if (test_motor1_steps == 0) disableMotor1();
  }

  if (test_motor2_steps > 0) {
    bool canMove = true;
    if (test_motor2_dir == 1 && motor2_position >= MOTOR2_MAX_STEPS) canMove = false;
    if (test_motor2_dir == -1 && motor2_position <= MOTOR2_HOME) canMove = false;

    if (canMove) {
      enableMotor2();
      digitalWrite(DIR_PIN2, (test_motor2_dir == 1) ? LOW : HIGH);
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);
      motor2_position += test_motor2_dir;
      motor2_needs_save = true;
      last_motor2_move_time = millis();
      last_any_mode_active_time = millis();
    }

    delayMicroseconds(MIN_DELAY_US);
    test_motor2_steps--;
    
    if (test_motor2_steps == 0) disableMotor2();
  }
}

// 수동 제어 모드에서 연속 스텝 구동
void driveManualMotors() {
  const long MANUAL_STEP_DELAY = MIN_DELAY_US;

  // 모터1 수동 이동
  if (motor1_manual_moving) {
    bool canMove = true;
    if (motor1_manual_dir == 1 && motor1_position <= MOTOR1_MIN_STEPS) canMove = false;
    if (motor1_manual_dir == 2 && motor1_position >= MOTOR1_MAX_STEPS) canMove = false;

    if (canMove) {
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN1, LOW);
      motor1_position += (motor1_manual_dir == 1) ? -1 : 1;
      last_any_mode_active_time = millis();
      delayMicroseconds(MANUAL_STEP_DELAY);
    }
  }

  // 모터2 수동 이동
  if (motor2_manual_moving) {
    bool canMove = true;
    if (motor2_manual_dir == 1 && motor2_position <= MOTOR2_HOME) canMove = false;
    if (motor2_manual_dir == 2 && motor2_position >= MOTOR2_MAX_STEPS) canMove = false;

    if (canMove) {
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);
      motor2_position += (motor2_manual_dir == 1) ? -1 : 1;
      motor2_needs_save = true;
      last_motor2_move_time = millis();
      last_any_mode_active_time = millis();
      delayMicroseconds(MANUAL_STEP_DELAY);
    }
  }
}

// 자동 왕복 회전 모드 제어
void rotationMode() {
  if (!rotation_mode) return;

  bool hit_limit = false;
  if (rotation_direction == 1 && motor1_position <= MOTOR1_MIN_STEPS) hit_limit = true;
  if (rotation_direction == 2 && motor1_position >= MOTOR1_MAX_STEPS) hit_limit = true;

  if (hit_limit) {
    disableMotor1(); // 리밋에서 잠시 전원 차단
    delay(1000);  
    rotation_direction = (rotation_direction == 1) ? 2 : 1;
    digitalWrite(DIR_PIN1, (rotation_direction == 1) ? HIGH : LOW);
    enableMotor1();
    return;
  }

  digitalWrite(STEP_PIN1, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN1, LOW);
  motor1_position += (rotation_direction == 1) ? -1 : 1;
  last_any_mode_active_time = millis();
  delayMicroseconds(60); 
}

// 자연풍 모드에서 BLDC 속도 레벨 순환
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

// 페이스 트래킹 제어 (1/32 스텝용 부드러운 PID+가속/감속)
void faceTrackingControl() {
  if (!facetracking_enabled) return;

  unsigned long now = micros();
  unsigned long now_ms = millis();

  static float smooth_delay_1 = MAX_DELAY_US;
  static float smooth_delay_2 = MAX_DELAY_US;
  const float ACCEL_FACTOR = 0.15; // 속도 변경 스무딩 계수

  // ===== Motor 1 (X축) =====
  int error1 = target_motor1_pos - motor1_position;
  
  if (abs(error1) <= TRACKING_DEADZONE) {
    // 데드존: 적분 서서히 줄이고 속도 완만히 감소
    if (integral_error_1 > 0) integral_error_1 -= 0.5;
    else if (integral_error_1 < 0) integral_error_1 += 0.5;
    smooth_delay_1 = (smooth_delay_1 * 0.98) + (MAX_DELAY_US * 0.02);
  } else {
    integral_error_1 += error1;
    integral_error_1 = constrain(integral_error_1, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float control_signal = (Kp * error1) + (Ki * integral_error_1);
    long target_delay = map((long)abs(control_signal), 0, 1000, MAX_DELAY_US, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, MAX_DELAY_US);
    smooth_delay_1 = (smooth_delay_1 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);
    
    if (now - lastStepTime1 >= (long)smooth_delay_1) {
      int dir = (error1 > 0) ? 1 : -1;
      
      // 리밋 도달 시 스텝만 차단, 전원은 유지
      bool canMove = true;
      if (dir == 1 && motor1_position >= MOTOR1_MAX_STEPS) {
        canMove = false;
        integral_error_1 = 0;
      }
      if (dir == -1 && motor1_position <= MOTOR1_MIN_STEPS) {
        canMove = false;
        integral_error_1 = 0;
      }

      if (canMove) {
        digitalWrite(DIR_PIN1, (dir == 1) ? LOW : HIGH);
        digitalWrite(STEP_PIN1, HIGH);
        delayMicroseconds(2);
        digitalWrite(STEP_PIN1, LOW);
        motor1_position += dir;
        last_any_mode_active_time = now_ms;
      }
      lastStepTime1 = now;
    }
  }

  // ===== Motor 2 (Y축) =====
  int error2 = target_motor2_pos - motor2_position;
  
  if (abs(error2) <= TRACKING_DEADZONE) {
    if (integral_error_2 > 0) integral_error_2 -= 0.5;
    else if (integral_error_2 < 0) integral_error_2 += 0.5;
    smooth_delay_2 = (smooth_delay_2 * 0.98) + (MAX_DELAY_US * 0.02);
  } else {
    integral_error_2 += error2;
    integral_error_2 = constrain(integral_error_2, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float control_signal = (Kp * error2) + (Ki * integral_error_2);
    long target_delay = map((long)abs(control_signal), 0, 1000, MAX_DELAY_US, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, MAX_DELAY_US);
    smooth_delay_2 = (smooth_delay_2 * (1.0 - ACCEL_FACTOR)) + (target_delay * ACCEL_FACTOR);

    if (now - lastStepTime2 >= (long)smooth_delay_2) {
      int dir = (error2 > 0) ? 1 : -1;
      
      bool canMove = true;
      if (dir == 1 && motor2_position >= MOTOR2_MAX_STEPS) {
        canMove = false;
        integral_error_2 = 0;
      }
      if (dir == -1 && motor2_position <= MOTOR2_HOME) {
        canMove = false;
        integral_error_2 = 0;
      }

      if (canMove) {
        digitalWrite(DIR_PIN2, (dir == 1) ? LOW : HIGH);
        digitalWrite(STEP_PIN2, HIGH);
        delayMicroseconds(2);
        digitalWrite(STEP_PIN2, LOW);
        motor2_position += dir;
        motor2_needs_save = true;
        last_motor2_move_time = now_ms;
        last_any_mode_active_time = now_ms;
      }
      lastStepTime2 = now;
    }
  }
  
  // 디버그용 그래프 출력 (시간, 목표, 현재)
  if (graph_debug_mode && (now_ms - last_graph_print >= 10)) {
    Serial1.print(now_ms);
    Serial1.print(",");
    Serial1.print(target_motor1_pos);
    Serial1.print(",");
    Serial1.println(motor1_position);
    last_graph_print = now_ms;
  }
}

// 통합 전원 관리 (모드 단위 기준)
void checkMotorPowerOff() {
  unsigned long now_ms = millis();
  
  // 어떤 모드라도 동작 중이면 활성 상태로 간주
  bool any_mode_active = motor1_manual_moving || motor2_manual_moving || 
                         rotation_mode || facetracking_enabled || 
                         (test_motor1_steps > 0) || (test_motor2_steps > 0);
  
  if (!any_mode_active) {
    if (now_ms - last_any_mode_active_time >= IDLE_POWER_OFF_MS) {
      disableMotor1();
      disableMotor2();
    }
  }
}

void loop() {
  // 명령 및 디버그 처리
  processUartCommand();
  processDebugCommand();
  
  // 모드 우선순위에 따른 모터 제어
  if (motor1_manual_moving || motor2_manual_moving) driveManualMotors();
  else if (test_motor1_steps > 0 || test_motor2_steps > 0) driveTestMotors();
  else if (rotation_mode) rotationMode();
  else if (facetracking_enabled) faceTrackingControl();
  
  // 전원 관리
  checkMotorPowerOff();
  
  // BLDC 및 EEPROM 관련 보조 기능
  naturalWindMode();
  updateBldcSpeed();
  checkAutoSave();

  if (bldcSpeedLevel > 0) measureRPM();
}
