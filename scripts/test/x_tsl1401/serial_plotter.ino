// Quick Diagnostic for X-TSL1401 with Serial Plotter
const int PIN_SI=8, PIN_CLK=9, PIN_AO=A0;
unsigned long EXPOSURE_US = 3000;
const int CLK_DELAY_US = 2;
int vbuf[128];

void pulseCLK(){
  digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US);
}

void beginScan(){
  digitalWrite(PIN_SI, HIGH);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLK_DELAY_US);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_SI, LOW);
  delayMicroseconds(CLK_DELAY_US);
}

void setup(){
  pinMode(PIN_SI,OUTPUT); pinMode(PIN_CLK,OUTPUT);
  pinMode(PIN_AO,INPUT);
  digitalWrite(PIN_SI,LOW); digitalWrite(PIN_CLK,LOW);
  analogReference(DEFAULT);
  Serial.begin(115200);
}

void loop(){
  delayMicroseconds(EXPOSURE_US);
  beginScan();

  // 첫 픽셀 버림
  pulseCLK(); analogRead(PIN_AO);

  int vmin=1023, vmax=0;
  for(int i=0;i<128;i++){
    digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US);
    int v = analogRead(PIN_AO);
    vbuf[i]=v;
    if(v<vmin)vmin=v; if(v>vmax)vmax=v;
  }

  // === Serial Plotter용 출력 ===
  Serial.print(vmin); Serial.print(" "); Serial.println(vmax);

  // 자동 노출
  if(vmax>980 && EXPOSURE_US>200) EXPOSURE_US/=2;
  else if(vmax<200 && EXPOSURE_US<30000) EXPOSURE_US*=2;
}
