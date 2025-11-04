// TOF 센서 2개 이상 사용

#include <Wire.h>                   // I2C 통신용 기본 라이브러리
#include "Adafruit_VL53L0X.h"       // VL53L0X 센서용 Adafruit 래퍼 라이브러리

// ─────────────────────────────────────────────────────────────────────────────
// [하드웨어 연결 개요]
// - VL53L0X는 기본 I2C 주소가 0x29로 동일합니다.
// - 여러 개를 동시에 쓰려면 XSHUT(Shutdown) 핀으로 개별 센서를 순차적으로 켠 뒤,
//   부팅 직후 기본주소(0x29)일 때 "주소 변경"을 센서 내부 레지스터에 기록해야 함.
// - 이렇게 각 센서의 주소를 서로 다르게 만든 후(예: 0x30, 0x31) 동시에 사용.
//
// [XSHUT 동작]
// - XSHUT = LOW  → 센서 하드웨어 리셋(전원 차단과 유사, 내부 초기화됨)
// - XSHUT = HIGH → 센서 활성화(부팅 시작)
// ─────────────────────────────────────────────────────────────────────────────

// XSHUT 핀(센서1, 센서2) — 각 센서 보드의 XSHUT(또는 SHDN)과 연결
const int XSHUT1 = 2;
const int XSHUT2 = 3;

// 우리가 부여할 새 I2C 주소(7-bit 주소 표기)
// ※ VL53L0X 기본주소 0x29와 충돌하지 않도록 서로 다른 값 사용
const uint8_t ADDR1 = 0x30;
const uint8_t ADDR2 = 0x31;

// 센서 객체 2개 생성(각각 한 센서를 담당)
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while(!Serial){}                  // (USB CDC 보드용) 시리얼 포트 준비될 때까지 대기
  // ⚠ OpenRB/UNO 등 보드에 따라 이 대기가 불필요하거나 영원히 대기할 수 있음(보드별 상이)

  // XSHUT 핀을 출력으로 사용하고, 일단 LOW로 내려서 모든 센서를 "리셋 상태"로 만듦
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // 1) 모든 센서 OFF(하드 리셋 상태)
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);                        // 리셋 유지(수 ms면 충분, 여유로 10ms)

  // 2) 센서1만 ON → 기본주소(0x29)로 부팅 → 부팅 직후에 새 주소로 변경
  digitalWrite(XSHUT1, HIGH);       // 센서1 활성화(부팅 시작)
  delay(10);                        // 부팅 안정화 대기(수~수십 ms 권장)

  // begin(addr)는 해당 addr로 센서에 접속/초기화 시도
  // 여기서는 기본주소 0x29로 먼저 붙는다(부팅 직후 아직 주소 변경 전이므로)
  if (!lox1.begin(0x29)) {
    Serial.println(F("Sensor1 boot fail"));   // 배선/전원/I2C 풀업/주소 충돌 여부 확인 필요
    while(1);                                 // 치명적 에러 시 정지
  }

  // ── 주소 변경 단계 ─────────────────────────────────────────────────────────
  // (주의/애매함) Adafruit_VL53L0X 라이브러리 버전에 따라 setAddress() API가
  // 없을 수도 있습니다. 이 경우 Pololu용 VL53L0X 라이브러리를 사용하거나
  // Adafruit 소스 내부(ST API)로 직접 'DeviceAddress'를 바꿔야 합니다.
  // 아래 코드는 setAddress()가 존재한다는 가정 하에 작성되어 있습니다.
  lox1.setAddress(ADDR1);           // 이제 센서1은 0x30(ADDR1)으로 응답하게 됨

  // 3) 센서2 ON → 기본주소(0x29)로 부팅 → 새 주소로 변경(센서1과 동일한 절차)
  digitalWrite(XSHUT2, HIGH);       // 센서2 활성화
  delay(10);                        // 부팅 안정화 대기

  if (!lox2.begin(0x29)) {
    Serial.println(F("Sensor2 boot fail"));
    while(1);
  }
  lox2.setAddress(ADDR2);           // 이제 센서2는 0x31(ADDR2)으로 응답

  Serial.println(F("Both sensors ready (0x30, 0x31)."));
}

void loop() {
  VL53L0X_RangingMeasurementData_t m1, m2;  // 각 센서의 측정 결과를 담을 구조체

  // 단발 측정 API: rangingTest(결과 포인터, 디버그출력여부)
  //  - false: 디버그 출력 꺼짐(빠름)
  //  - true : 내부 ST API의 상세 로그가 시리얼로 찍힘(느림)
  lox1.rangingTest(&m1, false);
  lox2.rangingTest(&m2, false);

  // 결과 포맷 출력
  Serial.print(F("S1: "));
  // RangeStatus == 4 는 "out of range(거리 유효값 없음/위상 실패)"로 흔히 처리
  // 참고: ST 정의
  //   0: Range Valid
  //   1: Sigma Fail
  //   2: Signal Fail
  //   4: Out of Range(Phase Fail 포함) → 값 신뢰 불가
  if (m1.RangeStatus != 4) Serial.print(m1.RangeMilliMeter); // 유효 시 mm 단위 거리
  else                     Serial.print(F("OOR"));            // Out Of Range

  Serial.print(F("  |  S2: "));
  if (m2.RangeStatus != 4) Serial.print(m2.RangeMilliMeter);
  else                     Serial.print(F("OOR"));
  Serial.println(F(" (mm)"));

  // 측정 간격(주기) 설정
  // VL53L0X 기본 타이밍 버짓이 대략 수십 ms 수준이라 10ms로 너무 빠르게 돌면
  // 업데이트가 늦거나 같은 값이 반복될 수 있습니다.
  // 필요 시 30~50ms 정도로 늘려 실제 타이밍 버짓과 맞추면 더 안정적입니다.
  delay(10);
}
