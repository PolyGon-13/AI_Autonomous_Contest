// 센터일 때만 직진, 나머지는 정지

// X-TSL1401 → Serial Monitor에 "검은 부분만 #, 나머지 공백" 한 줄 출력
// + 센터(64) ±5 안에 검은 픽셀 있으면 CENTER, 아니면 좌/우 개수 비교
const int PIN_SI  = 8;
const int PIN_CLK = 9;
const int PIN_AO  = A0;

const int CLK_DELAY_US   = 2;
const unsigned long EXPOSURE_US = 3000;  // 노출(적분) 시간
const int MARGIN = 20;                   // 임계값 여유

// 센서 중앙과 허용 오차(±5)
const int CENTER_INDEX = 64;
const int CENTER_TOL   = 0;

//====================모터
#define left_A 10                // 모터드라이브 left_A 10번핀 설정
#define left_B 11               // 모터드라이브 left_B 11번핀 설정
#define right_A 5                // 모터드라이브 right_A 5번핀 설정
#define right_B 6               // 모터드라이브 right_B 6번핀 설정

int speed = 200;              // 모터스피드 200으로 변수 설정

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

void forward(){
  analogWrite(left_A, 0);     // 정방향 5초 동작
  analogWrite(left_B, speed);

  analogWrite(right_A, speed);     // 역방향 5초 동작
  analogWrite(right_B, 0);
}

void stop(){
  analogWrite(left_A, 0);     // 정방향 5초 동작
  analogWrite(left_B, 0);

  analogWrite(right_A, 0);     // 역방향 5초 동작
  analogWrite(right_B, 0);
}

void setup() {
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);

  analogReference(DEFAULT);    // UNO: 5V 기준

  pinMode (left_A, OUTPUT);     // 출력 핀모드 A_1A
  pinMode (left_B, OUTPUT);     // 출력 핀모드 A_1B
  pinMode (right_A, OUTPUT);     // 출력 핀모드 A_1A
  pinMode (right_B, OUTPUT);     // 출력 핀모드 A_1B
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
      if (i < CENTER_INDEX) {
        leftCount++;
      }  
      else{      
        rightCount++;
      }  
      if (i >= leftBound && i <= rightBound) { //64기준 ±?? 안에 픽셀이 있으면
        hasCenterDark = true;
      }
    }
  }

  const char* side = "CENTER";
  if (hasCenterDark) {
    side = "CENTER";  // 센터 감지되면 CENTER
    forward();           
  } else if (leftCount > rightCount) {
      side = "LEFT";               // 왼쪽이 더 크면 LEFT
      stop();
  } else if (rightCount > leftCount) {
      side = "RIGHT";   
      stop();
  } /*else {   //좌 우 갯수가 동률일시 실행할 else문 추가해도 됌
      side = "CENTER";
      forward();
  */}

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
