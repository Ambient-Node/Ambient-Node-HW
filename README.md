
# Ambient-Node-HW - Fan and Axis control

Seeeduino XIAO RP2040을 사용해 BLDC모터 와 스텝모터 2개를 이용해 얼굴 추적, 자동 회전, 자연풍, 수동 조작 기능을 구현한 Ambient node 하위 HW프로젝트

한림대학교 25년도 2학기 SW캡스톤디자인

## Demo & Hardware

| | | |
|:---:|:---:|:---:|
| <img src="media/pan-test1.gif" width="250"/> | <img src="media/pan-test2.gif" width="250"/> | <img src="media/tilt-test1.gif" width="250"/> |
| pan 테스트1 | pan 테스트2 | tilt 테스트1 |
| <img src="media/pan-tilt-test1.gif" width="250"/> | <img src="media/pan-tilt-test2.gif" width="250"/> | <img src="media/fan-test1.gif" width="250"/> |
| pan-tilt 테스트1 | pan-tilt 테스트2 | fan 테스트1 |
| <img src="media/fan-test2.gif" width="250"/> | <img src="media/fan-test3.gif" width="250"/> | <img src="media/fan-pan-tilt-test.gif" width="250"/> |
| fan 테스트2 | fan 테스트3 | fan-pan-tilt 테스트 |
| <img src="media/soldering-plan.jpg" width="250"/> | <img src="media/after-soldering.jpg" width="250"/> | <img src="media/internal-configuration.png" width="250"/> |
| 솔더링 플랜 | 솔더링 후 | 내부 구성 |

---


## 하드웨어 구성

- MCU: seeduino XIAO RP2040
  - 초기에는 Seeeduino XIAO SAMD21을 사용했으나, BLDC 선풍기 제어에 필요한 20~60kHz 대역 고주파 PWM을 안정적으로 출력하기 어려워 제어용 보드를 XIAO RP2040으로 변경
  - RP2040은 PWM 대역이 유연하여 BLDC 팬용 고주파 PWM 신호 발생에 적합

- 스텝모터 2축 구성  
  - 모터1 (PAN, 좌우)  
    - STEP: D1  
    - DIR: D10  
    - EN: D2 (HIGH = 비활성화, LOW = 활성화)  
    - 소프트웨어 위치 범위: -10000 ~ +10000 스텝
  - 모터2 (TILT, 상하)  
    - STEP: D4  
    - DIR: D3  
    - EN: D5 (HIGH = 비활성화, LOW = 활성화)  
    - 소프트웨어 위치 범위: 0 ~ 8500 스텝 (0 = HOME)

- BLDC 선풍기 구성 
  - PWM 제어: D8 사용. 약 30Khz 고주파 PWM을 출력하며 듀티를 조정해 속도를 제어
  - FG(홀 센서) 입력: D9 사용. 인터럽트로 펄스를 카운트하여 RPM 계산.

- 기타 인터페이스 구성 
  - Serial: USB 디버그 및 테스트 명령 수신에 사용. 
  - Serial1: 얼굴 인식/좌표 및 제어 명령 수신용 UART로 사용.

## 보드 변경 배경

- XIAO SAMD21 사용 시 제약사항 
  - 기본 PWM 주파수가 낮고 타이머/채널 제약이 존재하여 BLDC 팬이 요구하는 20~60kHz 고주파 PWM을 안정적으로 구현하기 어려움. 
  - 타이머 레지스터 직접 제어를 통해 이론상 수십 kHz PWM 생성은 가능하나, 시스템 타이머(millis, delay 등) 및 다른 기능과 충돌 가능성이 존재함.  
  - 여러 채널에서 동일한 주파수·해상도를 유지하기 어렵고, 실제 팬 구동 시 특정 구간에서 소음, 떨림, 불안정한 구동이 발생할 수 있음.

- XIAO RP2040 선택 이유
  - RP2040은 다수의 PWM 채널을 제공하며, 고주파 PWM을 안정적으로 생성할 수 있음.  
  - 본 프로젝트에서는 BLDC 팬 구동용 PWM을 20~60kHz 대역으로 설정하기 위해 보드를 XIAO RP2040으로 변경.
  - XIAO RP2040은 Seeeduino XIAO(SAMD21)와 동일한 핀아웃과 폼팩터를 가지므로, 기존 PCB 설계를 수정하지 않고 그대로 사용할 수 있음.  
  - 보드 변경 이후 BLDC, 스텝모터, 고속 시리얼 통신을 동시에 사용해도 전체 시스템이 안정적으로 동작함.


## 주요 기능

### 얼굴 추적(페이스 트래킹) 기능

- 얼굴 좌표를 Serial1로 수신하여 선풍기 헤드를 얼굴 방향으로 회전시키는 기능
- 카메라 해상도는 1920 x 1080 기준으로 동작
  
- 명령 형식(Serial1)
  - `P (x,y)` 형식을 사용
    - 예: `P (960,540)` 형태로 사용

- 동작 방식 
  - 입력된 x, y 좌표를 기준으로 PAN/TILT 목표 스텝(target_motor1_pos, target_motor2_pos)을 계산. 
  - P 제어와 딜레이 스무딩을 사용하여 모터 속도를 조정.
  - TRACKING_DEADZONE 범위 내에서는 출력 명령을 억제하여 중앙 부근에서의 미세 떨림을 방지.

- 추적 정지 방식 
  - `P X` 또는 `P x` 명령을 수신하면 페이스 트래킹이 비활성화됨.  
  - 스텝모터 EN 핀을 HIGH로 설정하여 전류를 차단하고, 모터2 현재 위치를 EEPROM에 저장.

### BLDC 풍속 제어 기능

- 고정 풍량 모드 
  - 명령 형식(Serial1): `S n` (0~5) 사용. 
    - 0: 정지 또는 최소 풍량 수준으로 동작.
    - 1~5: SPEED_LEVELS 테이블에 정의된 PWM 값으로 속도를 설정.

- 자연풍 모드
  - 명령 형식(Serial1):  
    - `N 1` : 자연풍 모드 활성화
    - `N 0` : 자연풍 모드 비활성화 
  - NATURAL_WIND_MIN_LEVEL ~ NATURAL_WIND_MAX_LEVEL 범위 내에서 풍속 레벨을 주기적으로 증감시키며 자연스러운 풍량 변화를 구현
  - setBldcSpeed()와 updateBldcSpeed()를 통해 targetBldcPwm까지 1씩 보간하여 목표 PWM으로 서서히 수렴하도록 구현

- FG 기반 RPM 측정
  - BLDC_FG_PIN 인터럽트에서 fgPulseCount를 증가시키는 방식으로 펄스를 카운트
  - FG 3펄스를 1회전으로 가정하여 1초마다 RPM을 계산

### 자동 왕복 회전 모드 (좌우 자동 스윙)

- 명령 형식(Serial1) 
  - `R 1` : 회전 모드 활성화
  - `R 0` : 회전 모드 비활성화

- 동작 방식
  - 모터1(PAN)을 MOTOR1_MIN_STEPS ~ MOTOR1_MAX_STEPS 범위 내에서 왕복 회전시키는 기능 
  - 한쪽 끝에 도달하면 ROTATION_PAUSE_MS 동안 정지 후 회전 방향을 반전
  - 회전 모드 활성화 시 얼굴 추적 및 수동 조작 모드는 자동으로 비활성화

### 수동 조작 모드 (리모컨/버튼 대응)

- 명령 형식(Serial1, `A d state`)  

  - 모터1 (좌우) 제어  
    - `A r 1` : 오른쪽으로 수동 이동 시작
    - `A r 0` : 오른쪽 이동 정지
    - `A l 1` : 왼쪽으로 수동 이동 시작
    - `A l 0` : 왼쪽 이동 정지

  - 모터2 (상하) 제어
    - `A u 1` : 위쪽으로 수동 이동 시작
    - `A u 0` : 위쪽 이동 정지
    - `A d 1` : 아래쪽으로 수동 이동 시작
    - `A d 0` : 아래쪽 이동 정지

- 특징
  - 수동 모드 진입 시 facetracking 및 rotation_mode 플래그를 자동으로 비활성화하여 수동 조작이 최우선으로 동작 
  - 각 모터는 소프트웨어 위치 제한(MIN/MAX/HOME)을 항상 체크하여 허용 범위 밖으로 이동하지 않도록 구현
  - 모터2는 수동 이동 후 motor2_needs_save 플래그를 설정하고, 일정 시간이 지난 후 EEPROM에 위치가 자동 저장

### 단발 테스트 이동 (USB Serial 전용)

- 명령 형식(USB Serial) 
  - `1 cw 1000`  
  - `2 ccw 500`

- 동작 방식
  - 입력 문자열에서 모터 번호, 방향(cw/ccw), 스텝 수를 파싱하여 test_motor1_steps, test_motor2_steps 변수에 기록
  - loop에서 driveTestMotors()를 통해 지정된 스텝 수만큼만 이동하도록 동작
  - 이동 중에도 MIN/MAX/HOME 위치 제한을 적용
  - 테스트 이동 종료 후 EN 핀을 HIGH로 설정하여 모터 전류를 차단

### 모터 전원 관리 및 위치 저장

- EN 핀 로직
  - EN = LOW : 모터 드라이버 활성화(전류 인가, 홀딩 토크 유지) 
  - EN = HIGH : 모터 드라이버 비활성화(전류 차단, 발열 감소)

- 자동 전원 OFF 기능  
  - 마지막 스텝 이후 ENABLE_OFF_DELAY_MS 이상 동작이 없을 경우 EN을 HIGH로 설정하여 모터 전류를 자동 차단  
  - 수동 조작, 테스트 이동, 회전 모드, 얼굴 추적이 모두 비활성화된 상태에서만 적용

- 모터2 위치 EEPROM 저장
  - motor2_position은 MotorPosition 구조체(위치 값 및 유효 플래그) 형태로 EEPROM에 저장
  - motor2_position 변경 시 motor2_needs_save 플래그가 설정
  - 마지막 이동 후 EEPROM_SAVE_DELAY_MS가 경과하면 savePositionToFlash()를 호출하여 자동 저장 
  - 부팅 시 EEPROM에서 데이터를 읽어 VALID_SIGNATURE를 확인하고, 유효 시 해당 위치로 복원되며, 유효하지 않을 경우 HOME(0)으로 초기화 후 저장

### 디버그 및 유틸리티 기능

- USB Serial (Serial)
  - 그래프 모드  
    - `graph` 명령으로 그래프 디버그 모드를 토글
    - 활성화 시 Serial1로 "Time,Target,Current" 포맷의 데이터를 주기적으로 출력  
  - EEPROM 유틸리티  
    - `clear` : motor2_position을 초기화하고 유효 플래그를 제거
    - `sethome` : 현재 motor2_position을 HOME(0)으로 설정하고 EEPROM에 저장

- UART (Serial1) 사용
  - 얼굴 좌표 및 모든 제어 명령을 Serial1로 수신
  - 버퍼에 여러 줄이 쌓인 경우 가장 마지막 유효 명령만 처리하도록 구현

## 제어 로직 요약

- 모터 제어 우선순위는 다음과 같음.
  1. 수동 모터 조작 (A …)  
  2. 단발 테스트 스텝 이동 (1/2 cw/ccw …)  
  3. 자동 왕복 회전 (R 1)  
  4. 얼굴 추적 (P (x,y))

- loop 처리 흐름
  - 각 프레임마다 processUartCommand(), processDebugCommand()를 호출하여 명령을 처리
  - 우선순위에 따라 driveManualMotors(), driveTestMotors(), rotationMode(), faceTrackingControl() 중 하나를 실행.
  - checkMotorPowerOff(), naturalWindMode(), updateBldcSpeed(), checkAutoSave()를 호출하여 전원 관리, 자연풍 모드, BLDC 속도 보정, EEPROM 저장을 수행.
  - BLDC가 구동 중인 경우 measureRPM() 함수로 RPM을 주기적으로 측정

## 빌드 및 사용 방법

- Arduino IDE 보드 설
  - 보드: Seeed XIAO RP2040 보드 패키지 선택(https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json)
  - 시리얼 속도:  
    - USB Serial(Serial): 921600 사용함.  
    - UART(Serial1): 921600 사용함.

- 배선 및 업로드 절차  
  - 핀 정의에 맞게 스텝모터 드라이버와 BLDC 모터를 XIAO RP2040에 연결
  - 스케치를 업로드한 후 USB Serial 및 Serial1을 통해 동작을 검증

### 테스트 예시

- USB Serial(PC)에서 테스트
  - `1 cw 500`  
  - `2 ccw 500`

- Serial1(외부 장치)에서 제어
  - 얼굴 좌표: `P (960,540)`  
  - 얼굴 추적 정지: `P X`  
  - 풍속 설정: `S 3`  
  - 자연풍 모드: `N 1`  
  - 회전 모드: `R 1`  
  - 수동 조작: `A r 1`, `A r 0`, `A u 1`, `A u 0` 등 사용

## 주의사항 및 팁

- EN 핀 HIGH가 비활성화(전류 차단) 상태이므로, 구동 중이 아닐 때 EN을 HIGH로 유지하면 모터 발열이 감소됨.  
- 얼굴 추적, 자동 회전, 수동 조작 모드는 동시에 동작하지 않도록 상호 배타적으로 설계됨.  
- 모터2는 틸트 각도를 유지하기 위해 EEPROM에 위치를 저장. EEPROM_SAVE_DELAY_MS를 통해 저장 간격을 조절하여 EEPROM 수명을 보호.  
 
