// 라인인식 및 벽 구간 통과

#include <Arduino.h>
#include <math.h>
#include <Dynamixel2Arduino.h>      // 다이나믹셀제어

#include <Wire.h>                   // I2C 통신용 기본 라이브러리
#include "Adafruit_VL53L0X.h"       // VL53L0X 센서용 Adafruit 래퍼 라이브러리

// -----------------------------[ 카메라 튜닝 상수 ]-----------------------------
const int PIN_SI  = 6;                  // TSL1401: Start Integrate 제어 핀
const int PIN_CLK = 7;                  // TSL1401: 픽셀 시프트용 CLK 핀
const int PIN_AO  = A0;                 // TSL1401: 아날로그 출력(밝기) 입력 핀

const int CLK_DELAY_US   = 5;           // CLK 하이/로우 사이 지연(us)
const unsigned long EXPOSURE_US = 2000; // 적분(노출) 시간(us)
const int MARGIN = 0;                   // 임계값 보정 여유
static uint32_t last_si_us = 0;         // 고정 적분 주기 유지용 타임스탬프
const int MIN_CONTRAST = 50;           // 최소 콘트라스트(vmax - vmin) 기준
const int SKIP_LEFT  = 5;               // 계산에서 제외할 왼쪽 픽셀 수
const int SKIP_RIGHT = 5;               // 계산에서 제외할 오른쪽 픽셀 수
const int CENTER_INDEX = 63;            // 센서 중앙 인덱스(0~127)
const int CENTER_TOL   = 0;             // CENTER 판정 허용 폭(±TOL)
const int CHUNK = 16;                   // 픽셀을 16개 단위로 묶어서 noInterrupts()로 잠깐 보호, 이 구간 안에서는 다른 ISR(타이머, I²C 등)이 끼어들 수 없음
const int MIN_DARK_PIXELS = 6;          // '목표' 픽셀 최소 개수 기준(이제 흰선 기준일 때 밝은 픽셀 수)
float decayRate = 0.98;                 // 적분항 감쇠 비율

const bool FOLLOW_WHITE = true;         // true  => 흰선(밝은 픽셀) 인식 / false => 검은선(어두운 픽셀) 인식

//--------------------------------[ PID제어 튜닝 상수 ]------------------------------------------
float error = 0.0f, prev_error = 0.0f;            //오차
float Kp = 3.0f, Ki = 0.25f, Kd = 0.5f;           // PID 상수
float integral = 0.0f, deriv = 0.0f;
float dt;                                         // 주기
unsigned long lastTime = 0;
float Ks = 0.1f;                                  // PID값 조향각에 따라 감소
const float PID_LIMIT = 10.0f;                    // PID 최대 절대값 제한(PID값이 너무 커지거나 작아지는거 방지용)
float candidate_integral;                         // 현재 미분값 구하기
float candidate_PID;                              // 위에서 구한 미분값으로 PID값 구하기
unsigned long now;                                // 현재 millis 저장할 변수
float PID;                                        // 최종 출력

float wall_steering_device;
float wall_error = 0.0f;
float wall_Kp = 3.0f;
float wall_P;

// -----------------------------[ 라인인식 변수 ]------------------------------
int vbuf[128];                                    // 센서 버퍼
float leftSum, rightSum, leftAvg, rightAvg;
int targetTotal ;                                 //총 인식 센서 갯수
int vmin_val = 1023, vmax_val = 0, T_val = 0;
bool hasCenterDark = false;
int leftCount = 0, rightCount = 0;
int leftBound = 0, rightBound = 0;
const char* side = "CENTER";                      // 결과 방향
float CENTER_LINE;
bool lineDetected = false;
int  darkTotal = 0;                               // 이름은 유지하지만 '목표 픽셀 수(흰선=밝음)'를 담음
int  contrastVal = 0;
int i_idx = 0, v_sample = 0;
char lineBuf[129];                                // ASCII 출력 버퍼

// =============================[ TOF 센서 ]=============================
// [하드웨어 연결 개요]
// - VL53L0X는 기본 I2C 주소가 0x29로 동일합니다.
// - 여러 개를 동시에 쓰려면 XSHUT(Shutdown) 핀으로 개별 센서를 순차적으로 켠 뒤,
//   부팅 직후 기본주소(0x29)일 때 "주소 변경"을 센서 내부 레지스터에 기록해야 함.
// - 이렇게 각 센서의 주소를 서로 다르게 만든 후(예: 0x30, 0x31) 동시에 사용.
//
// [XSHUT 동작]
// - XSHUT = LOW  → 센서 하드웨어 리셋(전원 차단과 유사, 내부 초기화됨)
// - XSHUT = HIGH → 센서 활성화(부팅 시작)

// XSHUT 핀(센서1, 센서2, 센서3) — 각 센서 보드의 XSHUT(또는 SHDN)과 연결
//const int XSHUT1 = 2; // 전방
const int XSHUT2 = 3; // 좌측
const int XSHUT3 = 4; // 우특

// 우리가 부여할 새 I2C 주소(7-bit 주소 표기)
// ※ VL53L0X 기본주소 0x29와 충돌하지 않도록 서로 다른 값 사용
//const uint8_t ADDR1 = 0x30;
const uint8_t ADDR2 = 0x31;
const uint8_t ADDR3 = 0x32;

VL53L0X_RangingMeasurementData_t m1, m2, m3;  // 각 센서의 측정 결과를 담을 구조체

// 센서 객체 2개 생성(각각 한 센서를 담당)
//Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


void tof(){
  // (수정) 비차단 갱신: 50ms마다 한 번만 측정 수행
  static uint32_t last_tof_ms = 0;
  const uint16_t TOF_PERIOD_MS = 50;
  uint32_t ms = millis();
  if (ms - last_tof_ms >= TOF_PERIOD_MS) {
    //lox1.rangingTest(&m1, false);
    lox2.rangingTest(&m2, false);
    lox3.rangingTest(&m3, false);

    last_tof_ms = ms;
  }
}


// =============================[ AX-12A 모터제어 ]=============================
// OpenRB DXL 포트 
#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1                    // OpenRB는 Half-duplex 내장 → DIR핀 불필요
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//  사용자 설정 
const uint8_t  ID_steering   = 16;             // 제어할 조향모터 ID (필요시 변경)
const uint8_t  ID_LEFT       = 3;              // 제어할 좌측모터 ID (필요시 변경)
const uint8_t  ID_RIGHT      = 4;              // 제어할 우측모터 ID (필요시 변경)
int            speed         = 600;            //직진 속도

const float    PROTOCOL = 1.0;             // AX-12A는 Protocol 1.0
const uint32_t BAUD     = 1000000;         // 1 Mbps

// 조향장치 최대 회전각
const float DEG_LEFT = 145.0f;
const float DEG_RIGHT = 160.0f;

//중심값
const float DEG_CENTER = 152.5f;

// PID제어로 조향장치 값 입력 변수
float steering_device = 0; 


// AX-12A Control Table 주소(Protocol 1.0)
const uint16_t ADDR_CW_ANGLE_LIMIT   = 6;   // 2 bytes
const uint16_t ADDR_CCW_ANGLE_LIMIT  = 8;   // 2 bytes
const uint16_t ADDR_TORQUE_ENABLE    = 24;  // 1 byte
const uint16_t ADDR_MOVING_SPEED     = 32;  // 2 bytes

// 레지스터 쓰기 헬퍼
void write16(uint8_t id, uint16_t addr, uint16_t value) {
  uint8_t buf[2] = {
    (uint8_t)(value & 0xFF),        // Low byte
    (uint8_t)((value >> 8) & 0xFF)  // High byte
  };
  dxl.write(id, addr, buf, 2);
}
void write8(uint8_t id, uint16_t addr, uint8_t value) {
  dxl.write(id, addr, &value, 1);
}

//휠 속도 방향 설정
// speed: -1023 ~ +1023 (0=정지)
// +값=CCW(0~1023), -값=CW(1024~2047)
void setWheelSpeed(uint8_t id, int16_t speed) {
  if (speed > 1023)  speed = 1023;
  if (speed < -1023) speed = -1023;

  uint16_t word = 0;
  if (speed > 0) {
    word = (uint16_t)speed;                 // CCW
  } else if (speed < 0) {
    word = (uint16_t)(1024 + (-speed));     // CW
  } else {
    word = 0;                               // STOP
  }
  write16(id, ADDR_MOVING_SPEED, word);
}

//통신 시작
bool tryBeginAndPing(uint32_t baud) {
  dxl.begin(baud);
  dxl.setPortProtocolVersion(PROTOCOL);
  delay(20);
  bool ok = true;
  //ok &= dxl.ping(ID_LEFT);
  //ok &= dxl.ping(ID_RIGHT);
  //ok &= dxl.ping(ID_steering);
  return ok;
}

void PID_forward(float PID){
  //조향장치 보호 위함
  steering_device  = roundf(constrain(DEG_CENTER - PID, DEG_LEFT, DEG_RIGHT));
  
  //조향장치 제어
  dxl.setGoalPosition(ID_steering, steering_device, UNIT_DEGREE);

  //후륜모터 전진
  setWheelSpeed(ID_LEFT, speed);
  setWheelSpeed(ID_RIGHT, -speed);
}

void stop(){
  //조향장치 제어
  dxl.setGoalPosition(ID_steering, DEG_CENTER, UNIT_DEGREE);

  //후륜모터 정지
  setWheelSpeed(ID_LEFT, 0);
  setWheelSpeed(ID_RIGHT, 0);
}

void avoid(){ // 장애물 회피
  stop();
}

void wall(){ // 벽 따라 주행 p제어 사용
 
  wall_error = m2.RangeMilliMeter - m3.RangeMilliMeter;
  wall_P = (wall_error * wall_Kp) * Ks;

  wall_steering_device = roundf(constrain(DEG_CENTER - wall_P, DEG_LEFT, DEG_RIGHT));

  //조향장치 제어
  dxl.setGoalPosition(ID_steering, wall_steering_device, UNIT_DEGREE);

  //후륜모터 전진
  setWheelSpeed(ID_LEFT, speed);
  setWheelSpeed(ID_RIGHT, -speed);

}

// =============================[ 카메라 ]====================================
// 캡처 루프를 16픽셀 청크로 나눠 noInterrupts()/interrupts() 적용한 버전
void Camera() {
  // --- 고정 적분 주기 유지용 타임스탬프 (함수 내부 정적 변수) ---
  static uint32_t last_si_us = 0;

  // 1) [고정 적분] 직전 SI 이후 경과 시간만큼 기다려 적분 시간(EXPOSURE_US) 고정
  uint32_t now_us = micros();
  uint32_t elapsed = now_us - last_si_us;
  if (elapsed < EXPOSURE_US) {
    delayMicroseconds(EXPOSURE_US - elapsed);
  }

  // 2) 프레임 시작(beginScan): SI High → CLK High → CLK Low → SI Low
  digitalWrite(PIN_SI, HIGH);
  // SI 상승 직후를 다음 적분의 기준 시각으로 저장
  last_si_us = micros();

  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_SI, LOW);
  delayMicroseconds(CLK_DELAY_US);

  // 3) 첫 픽셀 버림(데이터 안정화)
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(CLK_DELAY_US);
  (void)analogRead(PIN_AO);

  // 4) 128픽셀 읽기 + min/max 갱신 (청크 단위로 인터럽트 잠깐만 비활성화)
  vmin_val = 1023;
  vmax_val = 0;

  for (int base = 0; base < 128; base += CHUNK) {
    int end = base + CHUNK;
    if (end > 128) end = 128;

    noInterrupts(); // ----- 짧게 보호 구간 시작 -----
    for (int idx = base; idx < end; ++idx) {
      // CLK 토글
      digitalWrite(PIN_CLK, HIGH);
      delayMicroseconds(CLK_DELAY_US);
      digitalWrite(PIN_CLK, LOW);
      delayMicroseconds(CLK_DELAY_US);

      // 픽셀 샘플링
      v_sample = analogRead(PIN_AO);
      vbuf[idx] = v_sample;

      // min/max (스킵 구간 제외)
      if (idx >= SKIP_LEFT && idx < 128 - SKIP_RIGHT) {
        if (v_sample < vmin_val) vmin_val = v_sample;
        if (v_sample > vmax_val) vmax_val = v_sample;
      }
    }
    interrupts(); // ----- 보호 구간 종료 -----
  }

  // 5) 적응 임계값 계산
  contrastVal = (vmax_val - vmin_val);
  T_val = (vmin_val + vmax_val) / 2 - MARGIN;
  T_val = constrain(T_val, 0, 1023);
}



// =============================[ 라인 분석 ]==================================== 
void analyze_Line() {
  leftCount = 0;
  rightCount = 0;
  leftSum = 0;
  rightSum = 0;

  leftBound  = CENTER_INDEX - CENTER_TOL;
  rightBound = CENTER_INDEX + CENTER_TOL;

  for (i_idx = 0; i_idx < 128; i_idx++) {
    if (i_idx < SKIP_LEFT || i_idx >= 128 - SKIP_RIGHT) {
      continue;
    }

    if (contrastVal >= MIN_CONTRAST) {
      // === 변경 포인트: 흰선(밝음) 추종 시 > T_val, 검은선(어두움) 추종 시 < T_val
      bool isTarget = FOLLOW_WHITE ? (vbuf[i_idx] > T_val) : (vbuf[i_idx] < T_val);

      if (isTarget) {
        if (i_idx < CENTER_INDEX) {
          leftCount++;
          leftSum += i_idx;
        } else {
          rightCount++;
          rightSum += i_idx;
        }
      }
    }
  }

  targetTotal = leftCount + rightCount;
  lineDetected = (targetTotal >= MIN_DARK_PIXELS);

  // [안전] 분모 0 방지
  if (lineDetected) {
    CENTER_LINE = (leftSum + rightSum) / targetTotal;
  } else {
    CENTER_LINE = CENTER_INDEX;
  }

  // 출력 호환 위해 이름 유지(이제 '목표 픽셀 수' 의미)
  darkTotal = targetTotal;

  if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound) {
    side = "CENTER";
  } else if (leftCount > rightCount) {
    side = "LEFT";
  } else if (rightCount > leftCount) {
    side = "RIGHT";
  }
}

// =============================[ PID ]====================================
void PID_control() {
  // (7) dt 업데이트
  now = millis();
  dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 1e-3f;
  lastTime = now;

  // (8) 전체 라인 중심 vs 기준점만으로 에러 계산
  if (lineDetected) {
    // 라인 중심이 왼쪽이면 (+), 오른쪽이면 (-)
    error = CENTER_INDEX - CENTER_LINE;
  } else {
    // 라인 미검출 시 조향 과민 방지: 에러 0 처리 + 적분항 감쇠
    error = 0.0f;
  }

 
  // D 제어
  deriv = (error - prev_error) / dt;
  prev_error = error;

  // I제어 (Anti-windup)
  candidate_integral = integral + error * dt; //미분값 구하기
  candidate_PID = (error * Kp + candidate_integral * Ki + deriv * Kd) * Ks; //위에서 구한 미분값으로 PID값 구하기

  // candidate_PID가 리미트 안에 있을 때만 적분항 업데이트
  if (candidate_PID > -PID_LIMIT && candidate_PID < PID_LIMIT) {
      integral = candidate_integral;
  }

  // 미세 오차 구간 또는 미검출 시 적분항 붕괴(드리프트 방지)
  if (fabs(error) < 0.1f || !lineDetected) {
    integral *= decayRate;
  }


  PID = (error * Kp + integral * Ki + deriv * Kd) * Ks;
  PID = constrain(PID, -PID_LIMIT, PID_LIMIT);
}

// =============================[ 주행 로직 ]================================
void Drive(){ 
  if (lineDetected) {
    PID_forward(PID);           //라인따라 이동
  } else {
    if (m2.RangeMilliMeter < 500 || m3.RangeMilliMeter < 500){
      wall();
      side = "wall";            // 벽 따라 이동
    }else{
      stop();                   //라인인식 실패 
      side = "NO_LINE";
    }
  }
}

// =============================[ 카메라 각 픽셀 값 입력 ]================================
void pixel_data() {
  for (i_idx = 0; i_idx < 128; i_idx++) {
    if (!lineDetected) {
      if (i_idx == CENTER_INDEX) {
        lineBuf[i_idx] = 'I';
      } else {
        lineBuf[i_idx] = ' ';
      }
    } else {
      if (i_idx == CENTER_INDEX) {
        lineBuf[i_idx] = 'I';
      } else {
        // === 표시 기준도 분석과 동일
        bool isTarget = FOLLOW_WHITE ? (vbuf[i_idx] > T_val) : (vbuf[i_idx] < T_val);
        if (isTarget) {
          lineBuf[i_idx] = '#';
        } else {
          lineBuf[i_idx] = ' ';
        }
      }
    }
  }
  lineBuf[128] = '\0';
}

// =============================[ 시리얼 출력 ]================================
void print_Serial() {
  Serial.print(lineBuf);

  Serial.print(" error=");
  Serial.print(error);

  Serial.print(" PID=");
  Serial.print(PID);

  Serial.print(" steering=");
  Serial.print(steering_device);

  Serial.print(" lineDetected: ");
  Serial.print(lineDetected);

  Serial.print(" darkTotal=");   
  Serial.print(darkTotal);

  Serial.print(" contrast=");
  Serial.print(contrastVal);
  
  // 결과 포맷 출력
  // RangeStatus == 4 는 "out of range(거리 유효값 없음/위상 실패)"로 흔히 처리
  // 참고: ST 정의
  //   0: Range Valid
  //   1: Sigma Fail
  //   2: Signal Fail
  //   4: Out of Range(Phase Fail 포함) → 값 신뢰 불가
  /*
  Serial.print(F(" S1: "));
  if (m1.RangeStatus != 4){
    Serial.print(m1.RangeMilliMeter); // 유효 시 mm 단위 거리
  }else{
    Serial.print(F("OOR"));            // Out Of Range
  }                     
  */
  Serial.print(F("  |  S2: "));
  if (m2.RangeStatus != 4){
    Serial.print(m2.RangeMilliMeter);
  }else{
    Serial.print(F("OOR"));
  }                     

  Serial.print(F("  |  S3: "));
  if (m3.RangeStatus != 4){
    Serial.print(m3.RangeMilliMeter);
  }else{
    Serial.print(F("OOR"));
  }                     
  Serial.print(F(" (mm)"));

  // 측정 간격(주기) 설정
  // VL53L0X 기본 타이밍 버짓이 대략 수십 ms 수준이라 10ms로 너무 빠르게 돌면
  // 업데이트가 늦거나 같은 값이 반복될 수 있습니다.
  // 필요 시 30~50ms 정도로 늘려 실제 타이밍 버짓과 맞추면 더 안정적입니다.

  Serial.print(" wall_steering=");
  Serial.print(wall_steering_device);

  Serial.print(" side: ");
  Serial.println(side);
}

// =============================[ 초기화 ]====================================
void setup() {
  Serial.begin(115200);

  Wire.begin();           // I²C 시작
  Wire.setClock(400000);  // I2C 400kHz, TOF 응답 지연 완화

  // 센서 핀
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);  

  lastTime = millis();
  prev_error = error;
  
  // [ TOF 센서 ]
  // XSHUT 핀을 출력으로 사용하고, 일단 LOW로 내려서 모든 센서를 "리셋 상태"로 만듦
  //pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  // 1) 모든 센서 OFF(하드 리셋 상태)
  //digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(10);                        // 리셋 유지(수 ms면 충분, 여유로 10ms)

  // [주소 변경 단계]
    /*
  // 2) 센서1만 ON → 기본주소(0x29)로 부팅 → 부팅 직후에 새 주소로 변경
  digitalWrite(XSHUT1, HIGH);       // 센서1 활성화(부팅 시작)
  delay(10);                        // 부팅 안정화 대기(수~수십 ms 권장)

  // begin(addr)는 해당 addr로 센서에 접속/초기화 시도
  // 여기서는 기본주소 0x29로 먼저 붙는다(부팅 직후 아직 주소 변경 전이므로)
  if (!lox1.begin(0x29)) {
    Serial.println(F("Sensor1 boot fail"));   // 배선/전원/I2C 풀업/주소 충돌 여부 확인 필요
    while(1);                                 // 치명적 에러 시 정지
  }

  // 아래 코드는 setAddress()가 존재한다는 가정 하에 작성되어 있습니다.
  lox1.setAddress(ADDR1);           // 이제 센서1은 0x30(ADDR1)으로 응답하게 됨
  */
  // 3) 센서2 ON → 기본주소(0x29)로 부팅 → 새 주소로 변경(센서1과 동일한 절차)
  digitalWrite(XSHUT2, HIGH);       // 센서2 활성화
  delay(10);                        // 부팅 안정화 대기

  if (!lox2.begin(0x29)) {
    Serial.println(F("Sensor2 boot fail"));
    while(1);
  }
  lox2.setAddress(ADDR2);           // 이제 센서2는 0x31(ADDR2)으로 응답

  // 4) 센서3 ON → 기본주소(0x29)로 부팅 → 새 주소로 변경(센서1과 동일한 절차)
  digitalWrite(XSHUT3, HIGH);       // 센서3 활성화
  delay(10);                        // 부팅 안정화 대기

  if (!lox3.begin(0x29)) {
    Serial.println(F("Sensor3 boot fail"));
    while(1);
  }
  lox2.setAddress(ADDR3);           // 이제 센서2는 0x32(ADDR3)으로 응답

  Serial.println(F("Both sensors ready (0x30, 0x31, 0x32)."));

  // [ 다이나믹셀 ] 
  // ★ OpenRB에서도 0~1023 스케일로 고정
  analogReadResolution(10);   

  // OpenRB: DXL 전원 켜기
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(50);

  // 통신 시작 & Ping
  if (!tryBeginAndPing(BAUD)) {
    Serial.println("[ERROR] DXL ping 실패: ID/배선/전원/BAUD 확인");
    while (1) delay(500);
  }
  Serial.println("Ping OK");

  // 모드선택 (DC 모터로 설정)
  write16(ID_LEFT,  ADDR_CW_ANGLE_LIMIT, 0);
  write16(ID_LEFT,  ADDR_CCW_ANGLE_LIMIT, 0);
  write16(ID_RIGHT, ADDR_CW_ANGLE_LIMIT, 0);
  write16(ID_RIGHT, ADDR_CCW_ANGLE_LIMIT, 0);
  
  // 토크 ON
  write8(ID_LEFT, ADDR_TORQUE_ENABLE, 1);
  write8(ID_RIGHT, ADDR_TORQUE_ENABLE, 1);
  write8(ID_steering, ADDR_TORQUE_ENABLE, 1);
}

// =============================[ 루프 ]====================================
void loop() {
  Camera();               
  analyze_Line();
  PID_control();
  tof();                          
  Drive();                
  pixel_data();           
  print_Serial();         
  delay(15);              
}

// 위 코드 주석 정리 ver
/*
#include <Arduino.h>
#include <math.h>
#include <Dynamixel2Arduino.h> // 다이나믹셀제어 라이브러리
#include <Wire.h>             // I2C 통신용 기본 라이브러리
#include "Adafruit_VL53L0X.h" // VL53L0X 센서용 Adafruit 래퍼 라이브러리

// [ 카메라 튜닝 상수 ]
const int PIN_SI = 6;  // TSL1401: Start Integrate 제어 핀
const int PIN_CLK = 7; // TSL1401: 픽셀 시프트용 CLK 핀
const int PIN_AO = A0; // TSL1401: 아날로그 출력(밝기) 입력 핀

const int CLK_DELAY_US = 5; // CLK 하이/로우 사이 지연(us)
const unsigned long EXPOSURE_US = 2000; // 적분(노출) 시간(us)
const int MARGIN = 0; // 임계값 보정 여유
static uint32_t last_si_us = 0; // 고정 적분 주기 유지용 타임스탬프
const int MIN_CONTRAST = 50; // 최소 콘트라스트(vmax - vmin) 기준
const int SKIP_LEFT = 5; // 계산에서 제외할 왼쪽 픽셀 수
const int SKIP_RIGHT = 5;  // 계산에서 제외할 오른쪽 픽셀 수
const int CENTER_INDEX = 63; // 센서 중앙 인덱스(0~127)
const int CENTER_TOL = 0; // CENTER 판정 허용 폭(±TOL)
const int CHUNK = 16; // 픽셀을 16개 단위로 묶어서 noInterrupts()로 잠깐 보호, 이 구간 안에서는 다른 ISR(타이머, I²C 등)이 끼어들 수 없음
const int MIN_DARK_PIXELS = 6; // '목표' 픽셀 최소 개수 기준(이제 흰선 기준일 때 밝은 픽셀 수)
float decayRate = 0.98; // 적분항 감쇠 비율

// true  => 흰선(밝은 픽셀) 인식 
// false => 검은선(어두운 픽셀) 인식
const bool FOLLOW_WHITE = true; 

// [ PID제어 튜닝 상수 ]
float error = 0.0f, prev_error = 0.0f; // 오차
float Kp = 3.0f, Ki = 0.25f, Kd = 0.5f; // PID 상수
float integral = 0.0f, deriv = 0.0f;
float dt; // 주기
unsigned long lastTime = 0;
float Ks = 0.1f; // PID값 조향각에 따라 감소
const float PID_LIMIT = 10.0f; // PID 최대 절대값 제한(PID값이 너무 커지거나 작아지는거 방지용)
float candidate_integral; // 현재 미분값 구하기
float candidate_PID; // 위에서 구한 미분값으로 PID값 구하기
unsigned long now; // 현재 millis 저장할 변수
float PID; // 최종 출력

float wall_steering_device;
float wall_error = 0.0f;
float wall_Kp = 3.0f;
float wall_P;

// [ 라인인식 변수 ]
int vbuf[128]; // 센서 버퍼
float leftSum, rightSum, leftAvg, rightAvg;
int targetTotal; // 총 인식 센서 갯수
int vmin_val = 1023, vmax_val = 0, T_val = 0;
bool hasCenterDark = false;
int leftCount = 0, rightCount = 0;
int leftBound = 0, rightBound = 0;
const char *side = "CENTER"; // 결과 방향
float CENTER_LINE;
bool lineDetected = false;
int contrastVal = 0;
int i_idx = 0, v_sample = 0;
char lineBuf[129]; // ASCII 출력 버퍼

// [ TOF 센서 ]
// [하드웨어 연결 개요]
// - VL53L0X는 기본 I2C 주소가 0x29로 동일합니다.
// - 여러 개를 동시에 쓰려면 XSHUT(Shutdown) 핀으로 개별 센서를 순차적으로 켠 뒤,
//   부팅 직후 기본주소(0x29)일 때 "주소 변경"을 센서 내부 레지스터에 기록해야 함.
// - 이렇게 각 센서의 주소를 서로 다르게 만든 후(예: 0x30, 0x31) 동시에 사용.
//
// [XSHUT 동작]
// - XSHUT = LOW  → 센서 하드웨어 리셋(전원 차단과 유사, 내부 초기화됨)
// - XSHUT = HIGH → 센서 활성화(부팅 시작)

// XSHUT 핀(센서1, 센서2, 센서3) — 각 센서 보드의 XSHUT(또는 SHDN)과 연결
const int XSHUT1 = 2; // 전방
const int XSHUT2 = 3; // 좌측
const int XSHUT3 = 4; // 우특

// 우리가 부여할 새 I2C 주소(7-bit 주소 표기)
// ※ VL53L0X 기본주소 0x29와 충돌하지 않도록 서로 다른 값 사용
const uint8_t ADDR1 = 0x30;
const uint8_t ADDR2 = 0x31;
const uint8_t ADDR3 = 0x32;

VL53L0X_RangingMeasurementData_t m1, m2, m3; // TOF 측정 데이터 객체
// m1 : 전방
// m2 : 좌측
// m3 : 우측

// 센서 객체 2개 생성(각각 한 센서를 담당)
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void tof()
{
    // 비차단 갱신: 50ms마다 한 번만 측정 수행
    static uint32_t last_tof_ms = 0;
    const uint16_t TOF_PERIOD_MS = 50;
    uint32_t ms = millis();
    if (ms - last_tof_ms >= TOF_PERIOD_MS)
    {
        lox1.rangingTest(&m1, false);
        lox2.rangingTest(&m2, false);
        lox3.rangingTest(&m3, false);

        last_tof_ms = ms;
    }
}

//[ AX-12A 모터제어 ]
// OpenRB DXL 포트
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1 // OpenRB는 Half-duplex 내장 → DIR핀 불필요
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const uint8_t ID_steering = 16; // 제어할 조향모터 ID (필요시 변경)
const uint8_t ID_LEFT = 3; // 제어할 좌측모터 ID (필요시 변경)
const uint8_t ID_RIGHT = 4; // 제어할 우측모터 ID (필요시 변경)
int speed = 600; // 직진 속도

const float PROTOCOL = 1.0; // AX-12A는 Protocol 1.0
const uint32_t BAUD = 1000000; // 1 Mbps

// 조향장치 최대 회전각
const float DEG_LEFT = 145.0f;
const float DEG_RIGHT = 160.0f;

// 중심값
const float DEG_CENTER = 152.5f;

// PID제어로 조향장치 값 입력 변수
float steering_device = 0;

// AX-12A Control Table 주소(Protocol 1.0)
const uint16_t ADDR_CW_ANGLE_LIMIT = 6;  // 2 bytes
const uint16_t ADDR_CCW_ANGLE_LIMIT = 8; // 2 bytes
const uint16_t ADDR_TORQUE_ENABLE = 24;  // 1 byte
const uint16_t ADDR_MOVING_SPEED = 32;   // 2 bytes

// 레지스터 쓰기 헬퍼
void write16(uint8_t id, uint16_t addr, uint16_t value)
{
    uint8_t buf[2] = {
        (uint8_t)(value & 0xFF), // Low byte
        (uint8_t)((value >> 8) & 0xFF) // High byte
    };
    dxl.write(id, addr, buf, 2);
}
void write8(uint8_t id, uint16_t addr, uint8_t value)
{
    dxl.write(id, addr, &value, 1);
}

// 다이나믹셀 속도와 회전 방향 설정
void setWheelSpeed(uint8_t id, int16_t speed)
{
    // speed: -1023 ~ +1023 (0=정지)
    if (speed > 1023)
        speed = 1023;
    if (speed < -1023)
        speed = -1023;

    uint16_t word = 0;
    if (speed > 0)
    {
        word = (uint16_t)speed; // CCW (반시계 회전 : 0~1023)
    }
    else if (speed < 0)
    {
        word = (uint16_t)(1024 + (-speed)); // CW (시계 회전 : 1024~2047)
    }
    else
    {
        word = 0; // STOP
    }
    write16(id, ADDR_MOVING_SPEED, word);
}

// 다이나믹셀 통신 시작
bool tryBeginAndPing(uint32_t baud)
{
    dxl.begin(baud);
    dxl.setPortProtocolVersion(PROTOCOL);
    delay(20);
    bool ok = true;
    // ok &= dxl.ping(ID_LEFT);
    // ok &= dxl.ping(ID_RIGHT);
    // ok &= dxl.ping(ID_steering);
    return ok;
}

// PID 제어로 조향장치 제어 및 전진
void PID_forward(float PID)
{
    steering_device = roundf(constrain(DEG_CENTER - PID, DEG_LEFT, DEG_RIGHT)); // 조향장치 목표각 계산 및 제한 (DEG_CENTER - PID 값을 DEG_LEFT~DEG_RIGHT 범위로 제한)
    // roundf: 소수점 반올림 함수
    // constrain: 특정 범위로 값 제한 함수

    dxl.setGoalPosition(ID_steering, steering_device, UNIT_DEGREE); // 조향장치 제어

    // 후륜모터 전진
    setWheelSpeed(ID_LEFT, speed);
    setWheelSpeed(ID_RIGHT, -speed);
}

// 정지
void stop()
{
    dxl.setGoalPosition(ID_steering, DEG_CENTER, UNIT_DEGREE); // 조향장치를 중앙 위치로 복귀

    // 후륜모터 정지
    setWheelSpeed(ID_LEFT, 0);
    setWheelSpeed(ID_RIGHT, 0);
}

// 장애물 회피
void avoid()
{
    stop();
}

// 벽 따라 주행 P제어 사용
void wall()
{
    wall_error = m2.RangeMilliMeter - m3.RangeMilliMeter; // 왼쪽 센서 거리와 오른쪽 센서 거리 차이
    wall_P = (wall_error * wall_Kp) * Ks; // P제어로 조향 보정값 계산

    wall_steering_device = roundf(constrain(DEG_CENTER - wall_P, DEG_LEFT, DEG_RIGHT)); // 조향 목표각 계산 및 제한

    dxl.setGoalPosition(ID_steering, wall_steering_device, UNIT_DEGREE); // 조향장치 제어

    // 후륜모터 전진
    setWheelSpeed(ID_LEFT, speed);
    setWheelSpeed(ID_RIGHT, -speed);
}

void Camera()
{
    static uint32_t last_si_us = 0; // 시작 타임스탬프 기록

    // 2ms 동안 빛을 모으도록 주기를 맞춤
    uint32_t now_us = micros();
    uint32_t elapsed = now_us - last_si_us;
    if (elapsed < EXPOSURE_US)
    {
        delayMicroseconds(EXPOSURE_US - elapsed);
    }

    // TSL1401 센서 내부 초기화 및 프레임 시작
    digitalWrite(PIN_SI, HIGH);
    last_si_us = micros(); // SI HIGH 직후 타임스탬프 갱신
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_SI, LOW); // SI를 LOW로 내려서 빛 모으기 시작
    delayMicroseconds(CLK_DELAY_US);

    // 첫 픽셀 버림 (데이터 안정화)
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(CLK_DELAY_US);
    (void)analogRead(PIN_AO);

    // 4) 128픽셀 읽기 + min/max 갱신 (청크 단위로 인터럽트 잠깐만 비활성화)
    vmin_val = 1023;
    vmax_val = 0;
    for (int base = 0; base < 128; base += CHUNK)
    {
        int end = base + CHUNK;
        if (end > 128)
            end = 128;

        noInterrupts(); // ----- 짧게 보호 구간 시작 -----
        for (int idx = base; idx < end; ++idx)
        {
            // CLK 토글
            digitalWrite(PIN_CLK, HIGH);
            delayMicroseconds(CLK_DELAY_US);
            digitalWrite(PIN_CLK, LOW);
            delayMicroseconds(CLK_DELAY_US);

            // 픽셀 샘플링
            v_sample = analogRead(PIN_AO);
            vbuf[idx] = v_sample;

            // min/max (스킵 구간 제외)
            if (idx >= SKIP_LEFT && idx < 128 - SKIP_RIGHT)
            {
                if (v_sample < vmin_val)
                    vmin_val = v_sample;
                if (v_sample > vmax_val)
                    vmax_val = v_sample;
            }
        }
        interrupts(); // ----- 보호 구간 종료 -----
    }

    // 5) 적응 임계값 계산
    contrastVal = (vmax_val - vmin_val);
    T_val = (vmin_val + vmax_val) / 2 - MARGIN;
    T_val = constrain(T_val, 0, 1023);
}

// 라인의 중앙/좌/우 판정
void analyze_Line()
{
    leftCount = 0;
    rightCount = 0;
    leftSum = 0;
    rightSum = 0;

    // 중앙으로 간주할 픽셀 지정
    leftBound = CENTER_INDEX - CENTER_TOL;
    rightBound = CENTER_INDEX + CENTER_TOL;

    for (i_idx = 0; i_idx < 128; i_idx++)
    {
        if (i_idx < SKIP_LEFT || i_idx >= 128 - SKIP_RIGHT) // 좌우 5픽셀 제외
        {
            continue;
        }

        if (contrastVal >= MIN_CONTRAST)
        {
            // 흰선 추적 : > T_val
            // 검은선 추적 : < T_val
            bool isTarget = FOLLOW_WHITE ? (vbuf[i_idx] > T_val) : (vbuf[i_idx] < T_val);
            
            // 목표 픽셀 개수 및 합계 계산
            if (isTarget)
            {
                if (i_idx < CENTER_INDEX)
                {
                    leftCount++;
                    leftSum += i_idx;
                }
                else
                {
                    rightCount++;
                    rightSum += i_idx;
                }
            }
        }
    }

    targetTotal = leftCount + rightCount; // 목표 픽셀 총 개수
    lineDetected = (targetTotal >= MIN_DARK_PIXELS); // MIN_DARK_PIXELS 이상이면 라인 검출로 간주

    if (lineDetected) // 라인 검출 시
    {
        CENTER_LINE = (leftSum + rightSum) / targetTotal; // 검출값의 평균 인덱스를 중앙으로 설정
    }
    else
    {
        CENTER_LINE = CENTER_INDEX;
    }

    // 중앙/좌/우 판정
    if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound)
    {
        side = "CENTER";
    }
    else if (leftCount > rightCount)
    {
        side = "LEFT";
    }
    else if (rightCount > leftCount)
    {
        side = "RIGHT";
    }
}

void PID_control()
{
    // (7) dt 업데이트
    now = millis();
    dt = (now - lastTime) / 1000.0f;
    if (dt <= 0)
        dt = 1e-3f;
    lastTime = now;

    // (8) 전체 라인 중심 vs 기준점만으로 에러 계산
    if (lineDetected)
    {
        // 라인 중심이 왼쪽이면 (+), 오른쪽이면 (-)
        error = CENTER_INDEX - CENTER_LINE;
    }
    else
    {
        // 라인 미검출 시 조향 과민 방지: 에러 0 처리 + 적분항 감쇠
        error = 0.0f;
    }

    // D 제어
    deriv = (error - prev_error) / dt;
    prev_error = error;

    // I제어 (Anti-windup)
    candidate_integral = integral + error * dt;                               // 미분값 구하기
    candidate_PID = (error * Kp + candidate_integral * Ki + deriv * Kd) * Ks; // 위에서 구한 미분값으로 PID값 구하기

    // candidate_PID가 리미트 안에 있을 때만 적분항 업데이트
    if (candidate_PID > -PID_LIMIT && candidate_PID < PID_LIMIT)
    {
        integral = candidate_integral;
    }

    // 미세 오차 구간 또는 미검출 시 적분항 붕괴(드리프트 방지)
    if (fabs(error) < 0.1f || !lineDetected)
    {
        integral *= decayRate;
    }

    PID = (error * Kp + integral * Ki + deriv * Kd) * Ks;
    PID = constrain(PID, -PID_LIMIT, PID_LIMIT);
}

// 주행
void Drive()
{
    if (lineDetected) // 라인 검출 시
    {
        PID_forward(PID); // PID 제어로 주행
    }
    else // 라인 미검출 시
    {
        if (m2.RangeMilliMeter < 500 || m3.RangeMilliMeter < 500) // 좌우 장애물이 0.5m 이내에 있는 경우
        {
            wall(); // 벽 따라 주행
            side = "wall";
        }
        else // 라인 미검출 & 장애물 없음
        {
            stop(); // 정지
            side = "NO_LINE";
        }
    }
}

void pixel_data()
{
    for (i_idx = 0; i_idx < 128; i_idx++)
    {
        if (!lineDetected) // 라인 미검출 시
        {
            if (i_idx == CENTER_INDEX)
            {
                lineBuf[i_idx] = 'I'; // 중앙 표시
            }
            else
            {
                lineBuf[i_idx] = ' '; // 미검출 픽셀 빈칸
            }
        }
        else // 라인 검출 시
        {
            if (i_idx == CENTER_INDEX)
            {
                lineBuf[i_idx] = 'I'; // 중앙 표시
            }
            else
            {
                bool isTarget = FOLLOW_WHITE ? (vbuf[i_idx] > T_val) : (vbuf[i_idx] < T_val); // 밝은 픽셀 추적 모드인지 판단
                if (isTarget)
                {
                    lineBuf[i_idx] = '#'; // 목표 픽셀 표시
                }
                else
                {
                    lineBuf[i_idx] = ' ';
                }
            }
        }
    }
    lineBuf[128] = '\0';
}

// 시리얼 출력
void print_Serial()
{
    Serial.print(lineBuf); // 픽셀 데이터 출력

    Serial.print(" error=");
    Serial.print(error);

    Serial.print(" PID=");
    Serial.print(PID);

    Serial.print(" steering=");
    Serial.print(steering_device);

    Serial.print(" lineDetected: ");
    Serial.print(lineDetected);

    Serial.print(" contrast=");
    Serial.print(contrastVal);

    // RangeStatus
    // 0: Range Valid
    // 1: Sigma Fail
    // 2: Signal Fail
    // 4: Out of Range → 값 신뢰 불가

    // RangeMilliMeter : TOF 센서 측정 거리

    // 중앙 TOF
    Serial.print(F(" S1: "));
    if (m1.RangeStatus != 4)
    {
        Serial.print(m1.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("OOR"));
    }

    // 좌측 TOF
    Serial.print(F("  |  S2: "));
    if (m2.RangeStatus != 4)
    {
        Serial.print(m2.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("OOR"));
    }

    // 우측 TOF
    Serial.print(F("  |  S3: "));
    if (m3.RangeStatus != 4)
    {
        Serial.print(m3.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("OOR"));
    }
    Serial.print(F(" (mm)"));

    Serial.print(" wall_steering=");
    Serial.print(wall_steering_device);

    Serial.print(" side: ");
    Serial.println(side);
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(); // I²C 시작
    Wire.setClock(400000); // I2C 400kHz, TOF 응답 지연 완화

    // 센서 핀
    pinMode(PIN_SI, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_AO, INPUT);
    digitalWrite(PIN_SI, LOW);
    digitalWrite(PIN_CLK, LOW);

    lastTime = millis();
    prev_error = error;

    // [ TOF 센서 ]
    // XSHUT 핀을 출력으로 사용하고, 일단 LOW로 내려서 모든 센서를 "리셋 상태"로 만듦
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    // 1) 모든 센서 OFF(하드 리셋 상태)
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
    delay(10); // 리셋 유지(수 ms면 충분, 여유로 10ms)

    // begin(addr) : 해당 addr로 센서에 접속/초기화 시도
    // 전방 TOF 활성화
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    if (!lox1.begin(0x29))
    {
        Serial.println(F("Sensor1 boot fail"));
        while (1)
            ; // 에러 시 프로그램 정지
    }
    lox1.setAddress(ADDR1); // 전방 TOF가 0x30(ADDR1)으로 응답하도록 설정

    // 좌측 TOF 활성화
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    if (!lox2.begin(0x29))
    {
        Serial.println(F("Sensor2 boot fail"));
        while (1)
            ;
    }
    lox2.setAddress(ADDR2); // 좌측 TOF가 0x31(ADDR2)으로 응답하도록 설정
    
    // 우측 TOF 활성화
    digitalWrite(XSHUT3, HIGH); 
    delay(10);
    if (!lox3.begin(0x29))
    {
        Serial.println(F("Sensor3 boot fail"));
        while (1)
            ;
    }
    lox2.setAddress(ADDR3); // 우측 TOF가 0x32(ADDR3)으로 응답하도록 설정

    Serial.println(F("Both sensors ready (0x30, 0x31, 0x32)."));

    // [ 다이나믹셀 ]
    // ★ OpenRB에서도 0~1023 스케일로 고정
    analogReadResolution(10);

    // OpenRB: DXL 전원 켜기
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    delay(50);

    // 통신 시작 & Ping
    if (!tryBeginAndPing(BAUD))
    {
        Serial.println("[ERROR] DXL ping 실패: ID/배선/전원/BAUD 확인");
        while (1)
            delay(500);
    }
    Serial.println("Ping OK");

    // 모드선택 (DC 모터로 설정)
    write16(ID_LEFT, ADDR_CW_ANGLE_LIMIT, 0);
    write16(ID_LEFT, ADDR_CCW_ANGLE_LIMIT, 0);
    write16(ID_RIGHT, ADDR_CW_ANGLE_LIMIT, 0);
    write16(ID_RIGHT, ADDR_CCW_ANGLE_LIMIT, 0);

    // 토크 ON
    write8(ID_LEFT, ADDR_TORQUE_ENABLE, 1);
    write8(ID_RIGHT, ADDR_TORQUE_ENABLE, 1);
    write8(ID_steering, ADDR_TORQUE_ENABLE, 1);
}

void loop()
{
    Camera(); // TSL1401 카메라로 픽셀값 읽기
    analyze_Line(); // 라인 분석
    PID_control(); // PID 제어 계산
    tof(); // TOF 센서 측정값 갱신
    Drive(); // 주행 판단 
    pixel_data(); // 픽셀 데이터 문자열 생성
    print_Serial(); // 상태 시리얼 출력
    delay(15);
}
*/
