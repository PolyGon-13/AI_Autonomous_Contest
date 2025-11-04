// 라인트레이싱 코드 함수화

#include <Arduino.h>
#include <math.h>

// -----------------------------[ 하드웨어 튜닝 상수 ]-----------------------------
const int PIN_SI  = 8;                  // TSL1401: Start Integrate 제어 핀
const int PIN_CLK = 9;                  // TSL1401: 픽셀 시프트용 CLK 핀
const int PIN_AO  = A0;                 // TSL1401: 아날로그 출력(밝기) 입력 핀

const int CLK_DELAY_US   = 5;           // CLK 하이/로우 사이 지연(us)
const unsigned long EXPOSURE_US = 5000; // 적분(노출) 시간(us)
const int MARGIN = 0;                   // 임계값 보정 여유
const int MIN_CONTRAST = 12;            // 최소 콘트라스트(vmax - vmin) 기준
const int SKIP_LEFT  = 5;               // 계산에서 제외할 왼쪽 픽셀 수
const int SKIP_RIGHT = 5;               // 계산에서 제외할 오른쪽 픽셀 수
const int CENTER_INDEX = 63;            // 센서 중앙 인덱스(0~127)
const int CENTER_TOL   = 0;             // CENTER 판정 허용 폭(±TOL)
const int MIN_DARK_PIXELS = 6;          // '검은' 픽셀 최소 개수 기준
float decayRate = 0.98;                 // 적분항 감쇠 비율

//--------------------------------[ PID제어 튜닝 상수 ]------------------------------------------
float error = 0.0f, prev_error = 0.0f;            //오차
float leftSum, rightSum, leftAvg, rightAvg;
float Kp = 1.0f, Ki = 0.25f, Kd = 0.2f;           // PID 상수
float integral = 0.0f, deriv = 0.0f;
float dt;                                         // 주기
unsigned long lastTime = 0;
float PID;                                        // 최종 출력

// -----------------------------[ 라인인식 변수 ]------------------------------
int vbuf[128];                                    // 센서 버퍼
int vmin_val = 1023, vmax_val = 0, T_val = 0;
bool hasCenterDark = false;
int leftCount = 0, rightCount = 0;
int leftBound = 0, rightBound = 0;
const char* side = "CENTER";                      // 결과 방향
float CENTER_LINE;
bool lineDetected = false;
int  darkTotal = 0;
int  contrastVal = 0;
int i_idx = 0, v_sample = 0;
char lineBuf[129];                                // ASCII 출력 버퍼

// -----------------------------[ 초음파센서 ]-----------------------------------
#define TRIG_PIN 2
#define ECHO_PIN 3
float duration;
float distance;
float avg_distance;
bool object = false;

float Distance(int loop) {
  distance = 0;
  for (int i = 0; i < loop; i++) {   // 평균값 구하기
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance += duration * 0.034 / 2;   // 거리 계산 (cm)
  }

  avg_distance = distance / loop;
  return avg_distance;
}

// -----------------------------[ 모터 ]-----------------------------------
#define left_speed 10
#define left_A 11
#define left_B 12
#define right_speed 5
#define right_A 6
#define right_B 7

int left_pwm;                   // 왼쪽 모터 속도
int right_pwm;                  // 오른쪽 모터 속도
float speed = 150;              // 기본 속도(0~255)

void P_forward(float PID) {
  left_pwm  = roundf(constrain((float)speed - PID, 0.0f, 255.0f));
  right_pwm = roundf(constrain((float)speed + PID, 0.0f, 255.0f));

  digitalWrite(left_A, HIGH);
  digitalWrite(left_B, LOW);
  analogWrite(left_speed, left_pwm);

  digitalWrite(right_A, HIGH);
  digitalWrite(right_B, LOW);
  analogWrite(right_speed, right_pwm);
}

void stop() {
  digitalWrite(left_A, LOW);
  digitalWrite(left_B, LOW);
  analogWrite(left_speed, 0);

  digitalWrite(right_A, LOW);
  digitalWrite(right_B, LOW);
  analogWrite(right_speed, 0);
}

// =============================[ 1~5 카메라 ]====================================
void Camera() {

  // (유틸리티) SI 펄스 발생
  digitalWrite(PIN_SI, HIGH); 
  delayMicroseconds(1);
  digitalWrite(PIN_SI, LOW);  
  delayMicroseconds(CLK_DELAY_US);

  // (1) 지난 프레임 플러시
  for (i_idx = 0; i_idx < 129; i_idx++) {
    // inline void pulseCLK() 대체
    {
      digitalWrite(PIN_CLK, HIGH); 
      delayMicroseconds(CLK_DELAY_US);

      digitalWrite(PIN_CLK, LOW);  
      delayMicroseconds(CLK_DELAY_US);
    }
  }

  // (2) 노출(적분)
  delayMicroseconds(EXPOSURE_US);

  // (3) 프레임 시작
  // inline void generateSI_Pulse() 대체
  {
    digitalWrite(PIN_SI, HIGH); 
    delayMicroseconds(1);

    digitalWrite(PIN_SI, LOW);  
    delayMicroseconds(CLK_DELAY_US);
  }
  vmin_val = 1023;
  vmax_val = 0;

  // (4) 픽셀 읽기 + min/max
  for (i_idx = 0; i_idx < 128; i_idx++) {
    // inline void pulseCLK() 대체
    {
      digitalWrite(PIN_CLK, HIGH); 
      delayMicroseconds(CLK_DELAY_US);

      digitalWrite(PIN_CLK, LOW);  
      delayMicroseconds(CLK_DELAY_US);
    }

    v_sample = analogRead(PIN_AO);
    vbuf[i_idx] = v_sample;

    if (i_idx >= SKIP_LEFT && i_idx < 128 - SKIP_RIGHT) {
      if (v_sample < vmin_val) {
        vmin_val = v_sample;
      }
      if (v_sample > vmax_val) {
        vmax_val = v_sample;
      }
    }
  }

  contrastVal = (vmax_val - vmin_val);

  // (5) 적응 임계값
  T_val = (vmin_val + vmax_val) / 2 - MARGIN;
  if (T_val < 0) {
    T_val = 0;
  }
  if (T_val > 1023) {
    T_val = 1023;
  }
}

// =============================[ 6 라인 분석 ]==================================== 
void analyze_Line() {
  hasCenterDark = false;
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
      bool isDark = (vbuf[i_idx] < T_val);
      if (isDark) {
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

  darkTotal = leftCount + rightCount;
  lineDetected = (darkTotal >= MIN_DARK_PIXELS);

  // [안전] 분모 0 방지
  if (darkTotal > 0) {
    CENTER_LINE = (leftSum + rightSum) / (float)darkTotal;
  } else {
    CENTER_LINE = CENTER_INDEX;
  }

  if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound) {      
    side = "CENTER";
  } else if (leftCount > rightCount) {
    side = "LEFT";
  } else if (rightCount > leftCount) {
    side = "RIGHT";
  }
}

// =============================[ 7~9 PID ]====================================
void PID_control() {
  // (7) dt 업데이트
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) {
    dt = 1e-3;
  }
  lastTime = now;

  // (8) error 계산
  if (CENTER_LINE >= leftBound && CENTER_LINE <= rightBound) {
    error = 0;
  } else if (leftCount > rightCount) {
    leftAvg = leftSum / leftCount;
    error = CENTER_INDEX - leftAvg;
  } else if (leftCount < rightCount) {
    rightAvg = rightSum / rightCount;
    error = CENTER_INDEX - rightAvg;
  }

  // (9) PID 갱신
  integral += error * dt;

  if (fabs(error) < 0.1f) {
    integral *= decayRate;
  }

  if (!lineDetected) {                              //라인인식 불가시 적분제어값 감소
    error = 0.0f;
    integral *= decayRate;
  }

  deriv = (error - prev_error) / dt;
  prev_error = error;

  PID = error * Kp + integral * Ki + deriv * Kd;
}

// =============================[ 주행 로직 ]================================
void Drive() {
  distance = Distance(3);

  if (distance >= 15 && lineDetected) {
    P_forward(PID);
    object = false;
  } else if (distance <= 15 && lineDetected) {
    stop();
    side = "stop";
    object = true;
  } else if (distance >= 15 && !lineDetected) {
    stop();
    side = "NO_LINE";
    object = false;
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
        if (vbuf[i_idx] < T_val) {
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

  Serial.print(" left=");
  Serial.print(left_pwm);

  Serial.print(" right=");
  Serial.print(right_pwm);

  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.print(" cm");

  Serial.print(" object: ");
  Serial.print(object);

  Serial.print(" lineDetected: ");
  Serial.print(lineDetected);

  Serial.print(" darkTotal=");
  Serial.print(darkTotal);

  Serial.print(" contrast=");
  Serial.print(contrastVal);

  Serial.print(" side: ");
  Serial.println(side);
}

// =============================[ 초기화 ]====================================
void setup() {
  // 센서 핀
  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_AO, INPUT);
  digitalWrite(PIN_SI, LOW);
  digitalWrite(PIN_CLK, LOW);

  // 모터 핀
  pinMode(left_speed, OUTPUT);
  pinMode(left_A, OUTPUT);
  pinMode(left_B, OUTPUT);

  pinMode(right_speed, OUTPUT);
  pinMode(right_A, OUTPUT);
  pinMode(right_B, OUTPUT);

  // 초음파 핀
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastTime = millis();
  prev_error = error;

  Serial.begin(115200);
}

// =============================[ 루프 ]====================================
void loop() {
  Camera();               // 1~5
  analyze_Line();         // 6
  PID_control();          // 7~9
  Drive();                // 10
  pixel_data();           // 11
  print_Serial();         // 12
  delay(15);              // 페이싱
}
