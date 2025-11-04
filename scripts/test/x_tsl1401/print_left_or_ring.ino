// X-TSL1401 → Serial Monitor에 "검은 부분만 #, 나머지 공백" 한 줄 출력
// + 센터(64) ±5 안에 검은 픽셀 있으면 CENTER, 아니면 좌/우 개수 비교
const int PIN_SI  = 8;
const int PIN_CLK = 9;
const int PIN_AO  = A0;

const int CLK_DELAY_US   = 2;
const unsigned long EXPOSURE_US = 2000;  // 노출(적분) 시간
const int MARGIN = 20;                   // 임계값 여유

// 센서 중앙과 허용 오차(±5)
const int CENTER_INDEX = 64;
const int CENTER_TOL   = 0;

int vbuf[128];

inline void pulseCLK() {
  digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);  delayMicroseconds(CLK_DELAY_US);
}

inline void beginScan() {
  digitalWrite(PIN_SI, HIGH);
  digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_SI, LOW);   delayMicroseconds(CLK_DELAY_US);
}

void setup() {
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);

  analogReference(DEFAULT);    // UNO: 5V 기준
  Serial.begin(115200);
}

void loop() {
  // 1) 적분(노출)
  delayMicroseconds(EXPOSURE_US);

  // 2) 라인 스캔
  beginScan();
  pulseCLK();
  analogRead(PIN_AO); // 첫 픽셀 버림

  int vmin = 1023, vmax = 0;
  for (int i=0; i<128; i++) {
    digitalWrite(PIN_CLK, HIGH); delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK, LOW);  delayMicroseconds(CLK_DELAY_US);
    int v = analogRead(PIN_AO);
    vbuf[i] = v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }

  // 3) 적응 임계값
  int T = (vmin + vmax)/2 - MARGIN;
  if (T < 0)    T = 0;
  if (T > 1023) T = 1023;

  // 4) 판정: 먼저 64±5 안에 '검은 픽셀' 존재 여부 확인
  bool hasCenterDark = false;
  int leftCount = 0, rightCount = 0;

  int leftBound  = CENTER_INDEX - CENTER_TOL; // 59
  int rightBound = CENTER_INDEX + CENTER_TOL; // 69

  for (int i=0; i<128; i++) {
    bool isDark = (vbuf[i] < T);
    if (isDark) {
      if (i < CENTER_INDEX) leftCount++;
      else                  rightCount++;

      if (i >= leftBound && i <= rightBound) hasCenterDark = true;
    }
  }

  const char* side = "CENTER";
  if (hasCenterDark) {
    side = "CENTER";
  } else if (leftCount > rightCount) {
    side = "LEFT";
  } else if (rightCount > leftCount) {
    side = "RIGHT";
  } else {
    side = "CENTER";
  }

  // 5) 한 줄 ASCII 출력 (#=검은, 공백=밝은)
  static char line[129];
  for (int i=0; i<128; i++) {
    line[i] = (vbuf[i] < T) ? '#' : ' ';
  }
  line[128] = '\0';
  Serial.print(line);

  // 6) 상태 로그
  Serial.print(" side="); Serial.println(side);

  delay(15);
}
