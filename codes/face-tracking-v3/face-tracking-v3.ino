#include <EEPROM.h>

//김찐따코드
bool graph_debug_mode = false;
unsigned long last_graph_print = 0;

// =========================================================
// [시스템 설정] 1/32 마이크로스텝 (실크 스무스 추적 튜닝)
// =========================================================

#define CAM_WIDTH 1920
#define CAM_HEIGHT 1080

// [모터 범위]
const int MOTOR1_MIN_STEPS = -10000;
const int MOTOR1_MAX_STEPS = 10000;
const int MOTOR2_HOME = 0;
const int MOTOR2_MAX_STEPS = 8500;

// [속도 설정 - 핵심 수정]
// 추적 시에는 너무 빠르면 오히려 끊겨 보입니다.
// 최고 속도를 낮춰서(시간을 늘려서) 모터가 부드럽게 이어지도록 합니다.
#define MIN_DELAY_US 150           // 50 -> 150 (추적 시 최고 속도 제한)
#define MAX_DELAY_US 1200          // 600 -> 1200 (출발을 아주 부드럽게)

// [반응성 설정 - 핵심 수정]
// 값을 낮춰서 모터가 "틱틱"거리지 않고 "스으윽" 움직이게 함
const float Kp = 1.2;              // 4.0 -> 1.2 (매우 부드러운 반응)
const float Ki = 0.01;             // 0.05 -> 0.01 (적분 최소화)
const int INTEGRAL_LIMIT = 500;    

// [이동 비율]
// 1/32스텝이라도 너무 민감하면 떨립니다. 약간 둔감하게 설정.
const float PAN_GAIN = 1.8;        // 3.2 -> 1.8 
const float TILT_GAIN = 1.8;       

#define TRACKING_DEADZONE 25       // 데드존을 넓혀서 정지 시 떨림 방지

// [전원 관리]
#define ENABLE_OFF_DELAY_MS 50     

// 기타 설정
#define EEPROM_SAVE_DELAY_MS 3000
#define NATURAL_WIND_MIN_LEVEL 1
#define NATURAL_WIND_MAX_LEVEL 3
#define NATURAL_WIND_CHANGE_INTERVAL_MS 3000
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

// 구조체 및 전역 변수
typedef struct {
  int motor2_position;
  int valid;
} MotorPosition;

int motor2_position = 0;
int motor1_position = 0;
const int VALID_SIGNATURE = 12345;

bool motor2_needs_save = false;
unsigned long last_motor2_move_time = 0;

int bldcSpeedLevel = 0;
int currentBldcPwm = 255;
int targetBldcPwm = 255;
unsigned long lastBldcUpdate = 0;
const int SPEED_LEVELS[6] = {255, 176, 158, 138, 116, 70};

volatile unsigned long fgPulseCount = 0;
unsigned long lastRpmTime = 0;
int currentRPM = 0;

bool motor1_manual_moving = false;
bool motor2_manual_moving = false;
int motor1_manual_dir = 0;
int motor2_manual_dir = 0;

int test_motor1_steps = 0;
int test_motor1_dir = 0;
int test_motor2_steps = 0;
int test_motor2_dir = 0;

bool facetracking_enabled = false;
int target_motor1_pos = 0;
int target_motor2_pos = 0;

unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

float integral_error_1 = 0;
float integral_error_2 = 0;

unsigned long last_motor1_action_time = 0;
unsigned long last_motor2_action_time = 0;

unsigned long last_motor1_step_time = 0;
unsigned long last_motor2_step_time = 0;

bool rotation_mode = false;
int rotation_direction = 1;

bool natural_wind_mode = false;
unsigned long last_natural_wind_change = 0;
int natural_wind_target_level = 1;
bool natural_wind_increasing = true;

unsigned long last_rotation_step = 0;
bool rotation_pausing = false;
unsigned long rotation_pause_start = 0;

void countFgPulse() { fgPulseCount++; }

void setup() {
  pinMode(BLDC_PWM_PIN, OUTPUT);
  analogWriteFreq(30000);
  analogWriteRange(255);
  analogWrite(BLDC_PWM_PIN, 255); 

  Serial.begin(921600);
  Serial.setTimeout(2); 
  
  Serial1.begin(921600);
  Serial1.setTimeout(2);

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

  pinMode(BLDC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BLDC_FG_PIN), countFgPulse, RISING);

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

  Serial.println("Ready (Silk Smooth Tracking)");
  lastRpmTime = millis();
}

void savePositionToFlash() {
  MotorPosition dataToSave;
  dataToSave.motor2_position = motor2_position;
  dataToSave.valid = VALID_SIGNATURE;
  EEPROM.put(0, dataToSave);
  EEPROM.commit();
  Serial.println("Saved.");
}

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

void checkAutoSave() {
  if (motor2_needs_save && (millis() - last_motor2_move_time >= EEPROM_SAVE_DELAY_MS)) {
    savePositionToFlash();
    motor2_needs_save = false;
  }
}

void setBldcSpeed(int level) {
  level = constrain(level, 0, 5);
  targetBldcPwm = (level == 0) ? 255 : SPEED_LEVELS[level];
  bldcSpeedLevel = level;
}

void updateBldcSpeed() {
  if (millis() - lastBldcUpdate >= 6) {
    if (currentBldcPwm < targetBldcPwm) currentBldcPwm++;
    else if (currentBldcPwm > targetBldcPwm) currentBldcPwm--;
    analogWrite(BLDC_PWM_PIN, currentBldcPwm);
    lastBldcUpdate = millis();
  }
}

void measureRPM() {
  if (millis() - lastRpmTime >= 1000) {
    if (bldcSpeedLevel > 0) {
      currentRPM = (fgPulseCount * 60) / 3;
      fgPulseCount = 0;
    }
    lastRpmTime = millis();
  }
}

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
    else if (input.startsWith("S ")) {
      natural_wind_mode = false;
      setBldcSpeed(input.substring(2).toInt());
    }
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
    else if (input.startsWith("R ")) {
      int rMode = input.substring(2).toInt(); 
      facetracking_enabled = false;
      
      if (rMode == 1) {
        rotation_mode = true;
        rotation_direction = 1;
        digitalWrite(DIR_PIN1, HIGH);
        digitalWrite(ENABLE_PIN1, LOW);
      } else {
        rotation_mode = false;
        digitalWrite(ENABLE_PIN1, HIGH);
      }
    }
    else if (input.startsWith("A ")) {
      facetracking_enabled = false;
      rotation_mode = false;
      
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      if (firstSpace > 0 && secondSpace > 0) {
        char direction = input.charAt(firstSpace + 1);
        int state = input.substring(secondSpace + 1).toInt();
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

void processDebugCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input == "graph") {
      graph_debug_mode = !graph_debug_mode;
      if(graph_debug_mode) Serial1.println("Time,Target,Current");
      else Serial1.println("Graph Mode OFF");
      return;
    }

    if (input == "clear") {
      clearFlashMemory();
      return;
    }
    if (input == "sethome") {
      motor2_position = MOTOR2_HOME;
      savePositionToFlash();
      return;
    }
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

void driveTestMotors() {
  unsigned long now_ms = millis();
  
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

void driveManualMotors() {
  const long MANUAL_STEP_DELAY = 100; // 수동은 조금 빠르게 (100us)
  unsigned long now_ms = millis();

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

void rotationMode() {
  if (!rotation_mode) return;
  unsigned long now_ms = millis();

  bool hit_limit = false;
  if (rotation_direction == 1 && motor1_position <= MOTOR1_MIN_STEPS) hit_limit = true;
  if (rotation_direction == 2 && motor1_position >= MOTOR1_MAX_STEPS) hit_limit = true;

  if (hit_limit) {
    digitalWrite(ENABLE_PIN1, HIGH); 
    delay(1000);  
    rotation_direction = (rotation_direction == 1) ? 2 : 1;
    digitalWrite(DIR_PIN1, (rotation_direction == 1) ? HIGH : LOW);
    return;
  }

  digitalWrite(ENABLE_PIN1, LOW);
  digitalWrite(STEP_PIN1, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN1, LOW);
  motor1_position += (rotation_direction == 1) ? -1 : 1;
  last_motor1_step_time = now_ms;
  delayMicroseconds(60); 
}

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

// [핵심] 부드러운 추적 알고리즘 (Soft Tracking)
void faceTrackingControl() {
  if (!facetracking_enabled) return;

  unsigned long now = micros();
  unsigned long now_ms = millis();

  // 가변 속도 변수
  static float smooth_delay_1 = MAX_DELAY_US;
  static float smooth_delay_2 = MAX_DELAY_US;
  
  // 가속도 설정 (작을수록 속도가 천천히 변함 = 부드러움)
  // 0.02는 매우 부드럽고 묵직한 움직임
  const float ACCEL_FACTOR = 0.02; 

  // ===== Motor 1 (X축) =====
  int error1 = target_motor1_pos - motor1_position;
  
  bool hit_limit1 = false;
  if (error1 > 0 && motor1_position >= MOTOR1_MAX_STEPS) hit_limit1 = true;
  if (error1 < 0 && motor1_position <= MOTOR1_MIN_STEPS) hit_limit1 = true;

  if (abs(error1) <= TRACKING_DEADZONE || hit_limit1) {
    if (hit_limit1 || (now_ms - last_motor1_action_time > ENABLE_OFF_DELAY_MS)) {
      digitalWrite(ENABLE_PIN1, HIGH);
    }
    // 감속 (Deceleration)
    smooth_delay_1 = (smooth_delay_1 * 0.98) + (MAX_DELAY_US * 0.02);
    
    // 적분 감소
    if (integral_error_1 > 0) integral_error_1 -= 1;
    else if (integral_error_1 < 0) integral_error_1 += 1;

  } else {
    integral_error_1 += error1;
    integral_error_1 = constrain(integral_error_1, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    float control_signal = (Kp * error1) + (Ki * integral_error_1);
    
    // 속도 계산 (MIN_DELAY_US = 150us 제한)
    long target_delay = map((long)abs(control_signal), 0, 1000, MAX_DELAY_US, MIN_DELAY_US);
    target_delay = constrain(target_delay, MIN_DELAY_US, MAX_DELAY_US);
    
    // 가속 스무딩 적용
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

  // ===== Motor 2 (Y축) =====
  int error2 = target_motor2_pos - motor2_position;
  
  bool hit_limit2 = false;
  if (error2 > 0 && motor2_position >= MOTOR2_MAX_STEPS) hit_limit2 = true;
  if (error2 < 0 && motor2_position <= MOTOR2_HOME) hit_limit2 = true;

  if (abs(error2) <= TRACKING_DEADZONE || hit_limit2) {
    if (hit_limit2 || (now_ms - last_motor2_action_time > ENABLE_OFF_DELAY_MS)) {
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
  
  if (graph_debug_mode && (now_ms - last_graph_print >= 10)) {
    Serial1.print(now_ms);
    Serial1.print(",");
    Serial1.print(target_motor1_pos);
    Serial1.print(",");
    Serial1.println(motor1_position);
    last_graph_print = now_ms;
  }
}

// 통합 전원 관리 함수
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
  processUartCommand();
  processDebugCommand();
  
  if (motor1_manual_moving || motor2_manual_moving) driveManualMotors();
  else if (test_motor1_steps > 0 || test_motor2_steps > 0) driveTestMotors();
  else if (rotation_mode) rotationMode();
  else if (facetracking_enabled) faceTrackingControl();
  
  checkMotorPowerOff();
  
  naturalWindMode();
  updateBldcSpeed();
  checkAutoSave();

  if (bldcSpeedLevel > 0) measureRPM();
}