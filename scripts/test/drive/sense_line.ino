// 라인트레이싱 + 라인 유무 감지

// X-TSL1401 → Serial Monitor에 "검은 부분만 #, 나머지 공백" 한 줄 출력
// + 센터(64) ±TOL 안에 검은 픽셀 있으면 CENTER, 아니면 좌/우 개수 비교
// + 가장자리 노이즈 보정: 왼쪽 SKIP_LEFT, 오른쪽 SKIP_RIGHT는 '계산에서만' 제외

#include <Arduino.h>
#include <math.h>

// -----------------------------[ 하드웨어/튜닝 상수 ]-----------------------------
const int PIN_SI  = 8;                  // TSL1401: Start Integrate 제어 핀
const int PIN_CLK = 9;                  // TSL1401: 픽셀 시프트용 CLK 핀
const int PIN_AO  = A0;                 // TSL1401: 아날로그 출력(밝기) 입력 핀

const int CLK_DELAY_US   = 5;           // CLK 하이/로우 사이 지연(us) → 타이밍 여유 / 값의 오차가 클떄 큰수로 올리면서 테스트
const unsigned long EXPOSURE_US = 5000; // 적분(노출) 시간(us) → 밝기 스케일 영향 / 값 높아질수로 임계값도 높아짐, 너무 높아지면 과포화상태 발현
const int MARGIN = 0;                   // 임계값 보정 여유(작을수록 민감)

const int MIN_CONTRAST = 12;         // 최소 콘트라스트(vmax - vmin) 기준

const int SKIP_LEFT  = 5;               // 계산에서 제외할 왼쪽 픽셀 수(고정 암부 보정)
const int SKIP_RIGHT = 5;               // 계산에서 제외할 오른쪽 픽셀 수

const int CENTER_INDEX = 63;            // 센서 중앙 인덱스(0~127)
const int CENTER_TOL   = 0;             // CENTER 판정 허용 폭(±TOL)

const int MIN_DARK_PIXELS = 6;          // '검은' 픽셀 최소 개수 기준

float decayRate = 0.98;                 // 적분항 감쇠 비율 (1.0이면 변화 없음, 0.98이면 2% 감소)

// -----------------------------[ 초음파센서 ]-----------------------------------
#define TRIG_PIN 2                  //TRIG 핀 설정 (초음파 보내는 핀)
#define ECHO_PIN 3                  //ECHO 핀 설정 (초음파 받는 핀)

float duration;
float distance;
float avg_distance;

bool object = false;

long Distance(int loop) {
  //평균값 구하기
  distance = 0;
  for (int i=0; i <= loop; i++){
    // Trig 핀에 짧은 펄스 발생
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Echo 핀에 들어온 펄스 시간 측정 (μs 단위)
    duration = pulseIn(ECHO_PIN, HIGH);

    // 거리 계산 (소리 속도 = 340 m/s = 0.034 cm/μs, 왕복이므로 /2)
    distance += duration * 0.034 / 2;
  }
  avg_distance = distance / loop;

  return avg_distance;
}
// -----------------------------[ 모터 ]-----------------------------------
//좌측
#define left_speed 10           // 모터1사용 가능 제어핀과 동시에 속도제어핀 역할
#define left_A 11               // 방향제어를 위한 핀1
#define left_B 12               // 방향제어를 위한 핀2
//우측
#define right_speed 5           // 모터1사용 가능 제어핀과 동시에 속도제어핀 역할
#define right_A 6               // 방향제어를 위한 핀1
#define right_B 7               // 방향제어를 위한 핀2

int left_pwm;                   //왼쪽 모터 속도
int right_pwm;                  //오른쪽 모터 속도

float speed = 150;              // 모터 기본 속도(PWM, 0~255)

void P_forward(float PID) {
  // 전진
  left_pwm  = roundf(constrain((float)speed - PID, 0.0f, 255.0f)); // 최대 최소 고정 및 반올림
  right_pwm = roundf(constrain((float)speed + PID, 0.0f, 255.0f)); // 최대 최소 고정 및 반올림

  digitalWrite(left_A, HIGH);
  digitalWrite(left_B, LOW);
  analogWrite(left_speed, left_pwm);

  digitalWrite(right_A, HIGH);
  digitalWrite(right_B, LOW);
  analogWrite(right_speed, right_pwm);
}

void stop() {// 정지
  digitalWrite(left_A, LOW);
  digitalWrite(left_B, LOW);
  analogWrite(left_speed, 0);

  digitalWrite(right_A, LOW);
  digitalWrite(right_B, LOW);
  analogWrite(right_speed, 0);
}

//--------------------------------[PID제어]------------------------------------------
//error
float error;                 //error값 저장 변수
float prev_error;            // 이전 오차 (D제어에 사용)

float leftSum;
float rightSum;

float leftAvg;
float rightAvg;

//P제어
float Kp = 2.0f;             //P제어 변수

//I제어
float Ki = 0.25f;            //I제어 변수
float integral = 0.0;        // 누적 오차

//D제어
float Kd = 0.2f;             //D제어 변수
float deriv = 0.0f;          // 미분항

//시간 간격
float dt;              //코드 한번 도는데 걸리는 시간(적분, 미분하기 위해 계산)

// PID제어 값
float PID;               // 최종 출력

unsigned long lastTime = 0;

// -----------------------------[ 센서 버퍼 ]--------------------------------------
int vbuf[128];                          // 128픽셀 밝기 저장(0~1023)

// -----------------------------[ 전역 작업 변수 ]---------------------------------
int vmin_val = 1023;                    // 스킵 제외 구간 내 최소 밝기
int vmax_val = 0;                       // 스킵 제외 구간 내 최대 밝기
int T_val = 0;                          // 적응 임계값
bool hasCenterDark = false;             // CENTER 구간 내 '검은' 픽셀 존재 여부
int leftCount = 0;                      // CENTER 기준 왼쪽의 검은 픽셀 수
int rightCount = 0;                     // CENTER 기준 오른쪽의 검은 픽셀 수
int leftBound = 0;                      // CENTER_TOL 적용한 왼쪽 경계
int rightBound = 0;                     // CENTER_TOL 적용한 오른쪽 경계
const char* side = "CENTER";            // 판정 결과 문자열 포인터
float CENTER_LINE;                      // 검은선 라인의 중심

// (추가) 라인 검출 상태 변수
bool lineDetected = false;              // 이번 프레임 라인 검출 여부
int  darkTotal    = 0;                  // 검은 픽셀 총합
int  contrastVal  = 0;                  // 프레임 콘트라스트(vmax - vmin)

// 루프용 인덱스/임시 값(지역 선언 금지 조건 때문에 전역에 둠)
int i_idx = 0;                          // for 루프 인덱스 용도
int dmyIdx = 0;                         // 더미 루프 인덱스
int v_sample = 0;                       // 일시 샘플 값 저장

// 출력용 라인 버퍼(ASCII 시각화)
char lineBuf[129];                      // 128 문자 + 종단문자

// -----------------------------[ 유틸리티 ]--------------------------------------
inline void pulseCLK() {
  // TSL1401 픽셀 시프트: CLK High → 지연 → Low → 지연
  digitalWrite(PIN_CLK, HIGH); 
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);  
  delayMicroseconds(CLK_DELAY_US);
}

// 데이터시트에 맞춘 짧은 SI 펄스 (플러시/재시작용)
inline void generateSI_Pulse() {
  digitalWrite(PIN_SI, HIGH);
  delayMicroseconds(1); // 최소 펄스폭
  digitalWrite(PIN_SI, LOW);
  delayMicroseconds(CLK_DELAY_US);
}

// -----------------------------[ 초기화 ]-----------------------------------------
void setup() {
  // 센서 핀 설정
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);

  // 모터 핀 설정
  //좌측 사용 가능 제어핀 출력으로 설정
  pinMode(left_speed, OUTPUT);
  pinMode(left_A, OUTPUT); 
  pinMode(left_B, OUTPUT);
  //우측 사용 가능 제어핀 출력으로 설정
  pinMode(right_speed, OUTPUT);
  pinMode(right_A, OUTPUT); 
  pinMode(right_B, OUTPUT);

  //초음파센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastTime = millis();
  prev_error = error;

  // 시리얼 모니터
  Serial.begin(115200);
}

// -----------------------------[ 메인 루프 ]--------------------------------------
void loop() {

  // (1) 플러시: 지난 프레임 데이터 비우기
  generateSI_Pulse();                     // 첫 번째 SI
  for (i_idx = 0; i_idx < 128 + 1; i_idx++) {
    pulseCLK();                           // 쓰레기 픽셀 클럭아웃
  }

  // (2) 노출(적분) 시간
  delayMicroseconds(EXPOSURE_US);

  // (3) 재-시작(SI) 후 이번 프레임 본격 스캔
  generateSI_Pulse();                     // 두 번째 SI

  // min/max 초기화(이번 프레임용)
  vmin_val = 1023;
  vmax_val = 0;

  // (4) 128 픽셀 읽기: 각 픽셀마다 CLK 시프트 후 AO 샘플
  for (i_idx = 0; i_idx < 128; i_idx++) {
    pulseCLK();                           // 데이터 시프트
    v_sample = analogRead(PIN_AO);        // 0~1023
    vbuf[i_idx] = v_sample;

    // 스킵 구간을 제외한 범위에서만 min/max 반영
    if (i_idx >= SKIP_LEFT && i_idx < 128 - SKIP_RIGHT) {
      if (v_sample < vmin_val) { 
        vmin_val = v_sample; 
      }
      if (v_sample > vmax_val) { 
        vmax_val = v_sample; 
      }
    }
  }

  // 3) 적응 임계값 계산: (vmin+vmax)/2 - MARGIN
  T_val = (vmin_val + vmax_val) / 2 - MARGIN;
  if (T_val < 0)    { 
    T_val = 0; 
  }
  if (T_val > 1023) { 
    T_val = 1023; 
  }


  // 4) 판정: CENTER 구간 '검은 픽셀' 우선 → 없으면 좌/우 개수 비교
  hasCenterDark = false;
  leftCount = 0;
  rightCount = 0;

  //좌우 인덱스 합(평균 구하기 위함)
  leftSum = 0;
  rightSum = 0;

  leftBound  = CENTER_INDEX - CENTER_TOL; // 예: TOL=0 → CENTER_INDEX만
  rightBound = CENTER_INDEX + CENTER_TOL;

  for (i_idx = 0; i_idx < 128; i_idx++) {
    // 계산에서 가장자리 스킵(센서 메커니컬/광학 가장자리 잡음 보정)
    if (i_idx < SKIP_LEFT || i_idx >= 128 - SKIP_RIGHT) {
      continue;
    }

    // 최대 최소 값 차이 적을시 라인이 없다고 판단
    contrastVal = (vmax_val - vmin_val);

    if (contrastVal >= MIN_CONTRAST) {        
      bool isDark = (vbuf[i_idx] < T_val);   // 임계값보다 어두우면 '검은 픽셀'
      if (isDark) {
        if (i_idx < CENTER_INDEX) { 
          leftCount++; 
          leftSum += i_idx;      // 왼쪽 인덱스 합계 누적
        } else { 
          rightCount++; 
          rightSum += i_idx;     // 오른쪽 인덱스 합계 누적
        }
      }
    }
  }
  CENTER_LINE = (leftSum + rightSum) / (leftCount + rightCount); // 검은색 라인 중심 구하기

  // 라인 검출 여부 계산 및 보수 처리
  darkTotal   = leftCount + rightCount;
  lineDetected = (darkTotal >= MIN_DARK_PIXELS);

  // 판정 결과 (기존 로직 유지 후 미검출 시 표기만 추가)
  if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound) {
    side = "CENTER";   // 센터 감지
  } else if (leftCount > rightCount) {
    side = "LEFT";     // 왼쪽이 더 많음
  } else if (rightCount > leftCount) {
    side = "RIGHT";    // 오른쪽이 더 많음
  } else {
    // 좌/우 동률 → 기본 정책 유지(여기서는 아무것도 안 함)
  }

  //--------------------------------[ PID제어 ]------------------------------------------
  
  //error값 구하기 (원래 계산 유지)
  if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound){
    error = 0;
  }else if (leftCount > rightCount) {  
    leftAvg = leftSum / leftCount; 
    error = CENTER_INDEX - leftAvg; // 선이 좌측이면 error 양수
  } else if (leftCount < rightCount){
    rightAvg = rightSum / rightCount;
    error = CENTER_INDEX - rightAvg; // 선이 우측이면 error 음수
  } 



  //I제어
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;   // ms → 초 단위
  lastTime = now;
  integral += error * dt;

  // 적분항을 서서히 줄임 (에러값이 계속 0일때 와리가리 맊기 위함)
  if (fabs(error) < 0.1) {
    integral = integral * decayRate; 
  }

  // 라인 미검출 시: 보수 처리(에러 0, 적분 감쇠)
  if (!lineDetected) {
    error = 0.0f;
    integral = integral * decayRate;
  }

  //D제어
  deriv = (error - prev_error) / dt;
  prev_error = error;

  //PID제어 값 저장
  PID = error * Kp + integral * Ki + deriv * Kd ;
  
  //초음파 센서로 받은 거리
  distance = Distance(3);
  if (distance >= 15 && lineDetected){
      //모터 속도 제어
    P_forward(PID); // 모터제어
    object = false;
  } else if(distance <= 15 && lineDetected){ // 장애물 인식
    stop();
    side = "stop";
    object = true;
  } else if(distance >= 15 && !lineDetected){ // 장애물 인식
    stop();
    side = "NO_LINE";
    object = false;
  }


  // 5) 한 줄 ASCII 시각화 출력
  for (i_idx = 0; i_idx < 128; i_idx++) {
    // [ADD] 라인 미검출이면 '#' 전부 숨기고 공백 출력(센터 마커 'I'만 유지)
    if (!lineDetected) {                          // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      lineBuf[i_idx] = (i_idx == CENTER_INDEX) ? 'I' : ' ';   // [ADD]
    } else {                                      // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      if (i_idx == CENTER_INDEX) {
        lineBuf[i_idx] = 'I';              // 중앙 마커(시각화 편의)
      } else {
        lineBuf[i_idx] = (vbuf[i_idx] < T_val) ? '#' : ' ';
      }
    }
  }
  lineBuf[128] = '\0';
  
//--------------------------------[ 로그출력 ]------------------------------------------
  // 6) 상태 로그(간단)
  Serial.print(lineBuf);

  Serial.print(" error=");
  Serial.print(error);

  Serial.print(" PID=");
  Serial.print(PID);  

  Serial.print(" left=");
  Serial.print(left_pwm);

  Serial.print(" right=");
  Serial.print(right_pwm);  

  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.print(" cm");

  Serial.print(" object: ");
  Serial.print(object);  

  // (추가) 라인 검출 정보 출력
  Serial.print(" lineDetected: ");
  Serial.print(lineDetected);
  Serial.print(" darkTotal=");
  Serial.print(darkTotal);
  // ★ 참고용: 콘트라스트 값도 같이 보고 싶다면 아래 두 줄 유지
  Serial.print(" contrast=");
  Serial.print(contrastVal);

  Serial.print(" side: ");
  Serial.println(side);

  // 루프 페이싱(대략 ~60~100Hz 근처, 실제는 시리얼/연산 시간 포함)
  delay(15);
}
