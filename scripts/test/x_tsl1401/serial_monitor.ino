// Quick Diagnostic for X-TSL1401 on Arduino UNO
const int PIN_SI=8, PIN_CLK=9, PIN_AO=A0;
unsigned long EXPOSURE_US = 3000;  // 노출(적분) 시간
const int CLK_DELAY_US = 2;
int vbuf[128];

void pulseCLK(){ digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
                 digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US); }

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
  analogReference(DEFAULT); // (애매) UNO 5V 기준. 3.3V 구동 땐 EXTERNAL/INTERNAL 검토
  Serial.begin(115200);
}

void loop(){
  delayMicroseconds(EXPOSURE_US); // 적분시간
  beginScan();

  // 첫 1~2픽셀은 버림(ADC/샘플 정착용)
  pulseCLK(); analogRead(PIN_AO);

  int vmin=1023, vmax=0;
  for(int i=0;i<128;i++){
    // CLK 하강 뒤 약간 지난 시점에 샘플
    digitalWrite(PIN_CLK,HIGH); delayMicroseconds(CLK_DELAY_US);
    digitalWrite(PIN_CLK,LOW);  delayMicroseconds(CLK_DELAY_US);
    int v = analogRead(PIN_AO);
    vbuf[i]=v; if(v<vmin)vmin=v; if(v>vmax)vmax=v;
  }

  // 프레임 요약
  Serial.print("min=");Serial.print(vmin);
  Serial.print(" max=");Serial.print(vmax);
  Serial.print(" exposure_us=");Serial.println(EXPOSURE_US);

  // 시도 1: 노출 자동 조정(포화/저조도 방지)
  if(vmax>980 && EXPOSURE_US>200) EXPOSURE_US/=2;         // 포화 → 더 짧게
  else if(vmax<200 && EXPOSURE_US<30000) EXPOSURE_US*=2;  // 너무 어두움 → 더 길게
}
