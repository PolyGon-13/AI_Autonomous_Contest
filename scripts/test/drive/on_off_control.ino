// 라인 트레이싱 ON/OFF 제어

// X-TSL1401 → Serial Monitor에 "검은 부분만 #, 나머지 공백" 한 줄 출력
// + 센터(64) ±TOL 안에 검은 픽셀 있으면 CENTER, 아니면 좌/우 개수 비교
// + 가장자리 노이즈 보정: 왼쪽 SKIP_LEFT, 오른쪽 SKIP_RIGHT는 '계산에서만' 제외

#include <Arduino.h>

// -----------------------------[ 하드웨어/튜닝 상수 ]-----------------------------
const int PIN_SI  = 8;        // TSL1401: Start Integrate 제어 핀
const int PIN_CLK = 9;        // TSL1401: 픽셀 시프트용 CLK 핀
const int PIN_AO  = A0;       // TSL1401: 아날로그 출력(밝기) 입력 핀

const int CLK_DELAY_US   = 2;           // CLK 하이/로우 사이 지연(us) → 타이밍 여유
const unsigned long EXPOSURE_US = 3000; // 적분(노출) 시간(us) → 밝기 스케일 영향
const int MARGIN = 10;                  // 임계값 보정 여유(작을수록 민감)

const int SKIP_LEFT  = 10;              // 계산에서 제외할 왼쪽 픽셀 수(고정 암부 보정)
const int SKIP_RIGHT = 10;              // 계산에서 제외할 오른쪽 픽셀 수

const int CENTER_INDEX = 64;            // 센서 중앙 인덱스(0~127)
const int CENTER_TOL   = 0;             // CENTER 판정 허용 폭(±TOL)

// -----------------------------[ 모터 핀/속도 ]-----------------------------------
#define left_A 10                       // H-bridge 채널 (배선 극성에 맞게 사용)
#define left_B 11
#define right_A 5
#define right_B 6

int speed = 100;                        // 전진 기본 속도(PWM, 0~255)

// -----------------------------[ 센서 버퍼 ]--------------------------------------
int vbuf[128];                          // 128픽셀 밝기 저장(0~1023)

// -----------------------------[ 전역 작업 변수 ]---------------------------------
// ※ 요청: 중간/루프 내부에서 int 등 선언하지 않기 → 전역으로 모두 선언
int vmin_val = 1023;                    // 스킵 제외 구간 내 최소 밝기
int vmax_val = 0;                       // 스킵 제외 구간 내 최대 밝기
int T_val = 0;                          // 적응 임계값
bool hasCenterDark = false;             // CENTER 구간 내 '검은' 픽셀 존재 여부
int leftCount = 0;                      // CENTER 기준 왼쪽의 검은 픽셀 수
int rightCount = 0;                     // CENTER 기준 오른쪽의 검은 픽셀 수
int leftBound = 0;                      // CENTER_TOL 적용한 왼쪽 경계
int rightBound = 0;                     // CENTER_TOL 적용한 오른쪽 경계
const char* side = "CENTER";            // 판정 결과 문자열 포인터

// 루프용 인덱스/임시 값(지역 선언 금지 조건 때문에 전역에 둠)
int i_idx = 0;                          // for 루프 인덱스 용도
int dmyIdx = 0;                         // 더미 루프 인덱스
int v_sample = 0;                       // 일시 샘플 값 저장

// 출력용 라인 버퍼(ASCII 시각화)
char lineBuf[129];                      // 128 문자 + 종단문자

// -----------------------------[ 유틸리티 ]--------------------------------------
inline void pulseCLK() {
  // TSL1401 픽셀 시프트: CLK High → 지연 → Low → 지연
  digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);  delayMicroseconds(CLK_DELAY_US);
}

inline void beginScan() {
  // 새 프레임 샘플 시작 시퀀스: SI↑, CLK↑, CLK↓, SI↓
  digitalWrite(PIN_SI, HIGH);
  digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_SI, LOW);   delayMicroseconds(CLK_DELAY_US);
}

// -----------------------------[ 모터 제어 ]--------------------------------------
void forward() {
  // 전진: left_B, right_A에 PWM 인가 (배선 극성 가정)
  analogWrite(left_A, 0);
  analogWrite(left_B, speed);

  analogWrite(right_A, speed);
  analogWrite(right_B, 0);
}

void left_turn() {
  // 전진: left_B, right_A에 PWM 인가 (배선 극성 가정)
  analogWrite(left_A, 0);
  analogWrite(left_B, 0);

  analogWrite(right_A, speed);
  analogWrite(right_B, 0);
}

void right_turn() {
  // 전진: left_B, right_A에 PWM 인가 (배선 극성 가정)
  analogWrite(left_A, 0);
  analogWrite(left_B, speed);

  analogWrite(right_A, 0);
  analogWrite(right_B, 0);
}

void stop() {
  // 정지: 모든 PWM 0
  analogWrite(left_A, 0);
  analogWrite(left_B, 0);
  analogWrite(right_A, 0);
  analogWrite(right_B, 0);
}

// -----------------------------[ 초기화 ]-----------------------------------------
void setup() {
  // 센서 핀 설정
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);
  analogReference(DEFAULT);           // UNO: 5V 기준(기본)

  // 모터 핀 설정
  pinMode(left_A, OUTPUT);
  pinMode(left_B, OUTPUT);
  pinMode(right_A, OUTPUT);
  pinMode(right_B, OUTPUT);

  // 시리얼 모니터
  Serial.begin(115200);
}

// -----------------------------[ 메인 루프 ]--------------------------------------
void loop() {
  // 1) 적분(노출): 센서가 빛을 적분하는 시간
  delayMicroseconds(EXPOSURE_US);

  // 2) 라인 스캔 시작
  beginScan();
  pulseCLK();                 // 첫 픽셀 정렬용 추가 CLK
  (void)analogRead(PIN_AO);   // 첫 샘플 버림(ADC 안정화)

  // min/max 초기화(이번 프레임용)
  vmin_val = 1023;
  vmax_val = 0;

  // 2-1) 128 픽셀 읽기(각 픽셀마다 CLK 토글 후 AO 샘플)
  for (i_idx = 0; i_idx < 128; i_idx++) {
    // 센서 타이밍: CLK High/Low → 샘플
    digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK, LOW);  delayMicroseconds(CLK_DELAY_US);

    v_sample = analogRead(PIN_AO);   // 0~1023
    vbuf[i_idx] = v_sample;

    // 스킵 구간을 제외한 범위에서만 min/max 반영
    if (i_idx >= SKIP_LEFT && i_idx < 128 - SKIP_RIGHT) {
      if (v_sample < vmin_val) { vmin_val = v_sample; }
      if (v_sample > vmax_val) { vmax_val = v_sample; }
    }
  }

  // 3) 적응 임계값 계산: (vmin+vmax)/2 - MARGIN
  T_val = (vmin_val + vmax_val) / 2 - MARGIN;
  if (T_val < 0)    { T_val = 0; }
  if (T_val > 1023) { T_val = 1023; }

  // 4) 판정: CENTER 구간 '검은 픽셀' 우선 → 없으면 좌/우 개수 비교
  hasCenterDark = false;
  leftCount = 0;
  rightCount = 0;

  leftBound  = CENTER_INDEX - CENTER_TOL; // 예: TOL=0 → 64만
  rightBound = CENTER_INDEX + CENTER_TOL;

  for (i_idx = 0; i_idx < 128; i_idx++) {
    // 계산에서 가장자리 스킵(센서 메커니컬/광학 가장자리 잡음 보정)
    if (i_idx < SKIP_LEFT || i_idx >= 128 - SKIP_RIGHT) {
      continue;
    }

    bool isDark = (vbuf[i_idx] < T_val);   // 임계값보다 어두우면 '검은 픽셀'
    if (isDark) {
      if (i_idx < CENTER_INDEX) { leftCount++; }
      else                      { rightCount++; }

      if (i_idx >= leftBound && i_idx <= rightBound) {
        hasCenterDark = true;              // CENTER 영역에서 검은 픽셀 발견
      }
    }
  }

  // 판정 결과와 모터 동작
  if (hasCenterDark) {
    side = "CENTER";   // 센터 감지 → 전진
    forward();
  } else if (leftCount > rightCount) {
    side = "LEFT";     // 왼쪽이 더 많음 → (여기서는) 정지
    left_turn();
  } else if (rightCount > leftCount) {
    side = "RIGHT";    // 오른쪽이 더 많음 → (여기서는) 정지
    right_turn();
  } else {
    forward();
    // 좌/우 동률 → 기본 정책 유지(여기서는 아무것도 안 함)
    // side = "CENTER"; forward();  // 필요 시 활성화
  }

  // 5) 한 줄 ASCII 시각화 출력
  for (i_idx = 0; i_idx < 128; i_idx++) {
    if (i_idx == 63) {
      lineBuf[i_idx] = 'I';              // 중앙 마커(시각화 편의)
    } else {
      lineBuf[i_idx] = (vbuf[i_idx] < T_val) ? '#' : ' ';
    }

    // (옵션) 출력에서도 스킵 구간 비우고 싶으면 주석 해제
    // if (i_idx < SKIP_LEFT || i_idx >= 128 - SKIP_RIGHT) { lineBuf[i_idx] = ' '; }
  }
  lineBuf[128] = '\0';
  Serial.print(lineBuf);

  // 6) 상태 로그(간단)
  Serial.print(" side=");
  Serial.println(side);

  // 루프 페이싱(대략 ~60~100Hz 근처, 실제는 시리얼/연산 시간 포함)
  delay(15);
}
