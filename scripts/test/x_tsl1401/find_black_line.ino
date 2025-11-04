// X-TSL1401 → Serial Monitor에 "검은 부분만 #, 나머지 공백" 한 줄 출력
// 보드: Arduino UNO 예시
const int PIN_SI  = 8;
const int PIN_CLK = 9;
const int PIN_AO  = A0;

const int CLK_DELAY_US   = 2;
const unsigned long EXPOSURE_US = 2000;  // 노출(적분) 시간: 조명/속도에 맞게 조절
const int MARGIN = 20;                    // 적응 임계값 여유 (10~60에서 튜닝)

int vbuf[128];

inline void pulseCLK(){
  digitalWrite(PIN_CLK, HIGH); 
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);  
  delayMicroseconds(CLK_DELAY_US);
}

inline void beginScan(){
  digitalWrite(PIN_SI, HIGH);
  digitalWrite(PIN_CLK, HIGH); 
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_SI, LOW);   
  delayMicroseconds(CLK_DELAY_US);
}

void setup(){
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);

  analogReference(DEFAULT);    // UNO: 5V 기준 (3.3V 보드면 보드에 맞춰 변경)
  Serial.begin(115200);
}

void loop(){
  // 1) 적분(노출)
  delayMicroseconds(EXPOSURE_US);

  // 2) 라인 스캔
  beginScan();
  pulseCLK(); 
  analogRead(PIN_AO); // 첫 픽셀 버림(안정화)
  int vmin = 1023, vmax = 0;
  for(int i=0;i<128;i++){
    digitalWrite(PIN_CLK, HIGH); 
    delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK, LOW);  
    delayMicroseconds(CLK_DELAY_US);
    
    int v = analogRead(PIN_AO);
    vbuf[i] = v;
    if(v < vmin) vmin = v;
    if(v > vmax) vmax = v;
  }

  // 3) 적응 임계값 (프레임마다 자동)
  int T = (vmin + vmax)/2 - MARGIN;
  if(T < 0)    T = 0;
  if(T > 1023) T = 1023;

  // 4) 한 줄로 128칸 출력: 검은(어두운) 픽셀만 '#', 나머지는 공백
  static char line[129];
  for(int i=0;i<128;i++){
    if (vbuf[i] < T) {
      line[i] = '#';   // 픽셀 값이 임계값보다 작으면(=어두우면) '#' 출력
    } else {
      line[i] = ' ';   // 그 외(=밝으면) 공백 출력
    }
  }
  line[128] = '\0';
  Serial.println(line);

  // (선택) 디버깅용 상태 확인을 원하면 아래 주석 해제
  // Serial.print("vmin="); Serial.print(vmin);
  // Serial.print(" vmax="); Serial.print(vmax);
  // Serial.print(" T=");    Serial.println(T);

  delay(15); // 프레임 간격 (원하는 속도로 조정)
}
