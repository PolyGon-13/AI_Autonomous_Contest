// === ASCII Plot: profile + center line in Serial Monitor ===
const int PIN_SI=8, PIN_CLK=9, PIN_AO=A0;
const int CLK_DELAY_US=2;
const unsigned long EXPOSURE_US=2000;
int vbuf[128];

void pulseCLK(){ digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
                 digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US); }
void beginScan(){ digitalWrite(PIN_SI,HIGH); digitalWrite(PIN_CLK,HIGH);
                  delayMicroseconds(CLK_DELAY_US); digitalWrite(PIN_CLK,LOW);
                  digitalWrite(PIN_SI,LOW); delayMicroseconds(CLK_DELAY_US); }

void setup(){
  pinMode(PIN_SI,OUTPUT); pinMode(PIN_CLK,OUTPUT); pinMode(PIN_AO,INPUT);
  digitalWrite(PIN_SI,LOW); digitalWrite(PIN_CLK,LOW);
  analogReference(DEFAULT);
  Serial.begin(115200);
}

void loop(){
  delayMicroseconds(EXPOSURE_US);
  beginScan();
  pulseCLK(); analogRead(PIN_AO); // throw away first

  int vmin=1023, vmax=0;
  for(int i=0;i<128;i++){
    digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US);
    int v=analogRead(PIN_AO);
    vbuf[i]=v;
    if(v<vmin) vmin=v; if(v>vmax) vmax=v;
  }

  // 중심(무게중심)
  long sumW=0,sumD=0; int T=(vmin+vmax)/2 - 20;
  for(int i=0;i<128;i++){ int d=T - vbuf[i]; if(d<0) d=0; sumW += (long)i*d; sumD += d; }
  float pos = (sumD>0)? (float)sumW/sumD : 64.0;

  // === ASCII 막대 그리기 (너비 64칸으로 downsample) ===
  const int WIDTH=64;              // 화면 가로 글자수
  const int HEIGHT=20;             // 세로 해상도(줄 수)
  static char line[WIDTH+1];

  // 1) 128 → 64로 다운샘플(픽셀 2개 평균)
  int p[WIDTH];
  for(int x=0;x<WIDTH;x++){
    int i0 = 2*x, i1 = 2*x+1;
    int m = (vbuf[i0]+vbuf[i1])/2;
    p[x]=m;
  }

  // 2) 정규화(밝을수록 위, 어두울수록 아래로 보이게 반전)
  //    값이 낮은(검정선) 지점이 아래로 '골'처럼 내려가 보임
  for(int y=0;y<HEIGHT;y++){
    for(int x=0;x<WIDTH;x++){
      // 높이 결정: 어두울수록 아래(y가 큼) 찍히게
      // 층 기준값: v = vmin + (vmax-vmin)*(1 - y/(HEIGHT-1))
      float level = vmin + (vmax - vmin) * (1.0f - (float)y/(HEIGHT-1));
      line[x] = (p[x] <= level) ? '#' : ' ';
    }
    line[WIDTH]='\0';

    // 중심선 표시(세로선): pos(0..127) → 0..(WIDTH-1)
    int cx = (int)(pos * (WIDTH-1) / 127.0f + 0.5f);
    if(cx>=0 && cx<WIDTH) line[cx] = '|';

    Serial.println(line);
  }

  // 상태 출력
  Serial.print("vmin=");Serial.print(vmin);
  Serial.print(" vmax=");Serial.print(vmax);
  Serial.print(" pos="); Serial.println(pos);

  Serial.println(); // 프레임 간 빈 줄
  delay(30);
}
