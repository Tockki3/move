/****************************************************************
 * Jellibi AGV - 라인트레이싱 미세조정 코드 (Line Trace Tuner)
 *   · 라인을 따라 주행하면서 Kp / 속도를 실시간 조정
 *   · 정규화·P제어·drive() 로직이 베이스와 동일 → 값 그대로 복사
 *
 * [명령] (시리얼 모니터 9600, 입력창에 글자 전송)
 *   g : 주행 시작(라인트레이싱)     s : 정지
 *   1/2 : Kp  -0.005 / +0.005  (coarse)
 *   9/0 : Kp  -0.001 / +0.001  (fine)
 *   3/4 : basePower(속도) -5 / +5
 *   5/6 : maxPower -5 / +5      7/8 : minPower -5 / +5
 *   p : 현재 설정 출력
 *
 * [튜닝 가이드]
 *   · 지그재그(좌우 흔들림)·jerky → Kp 너무 큼  → 1/9 로 ↓
 *   · 곡선에서 못 따라가고 라인 이탈/코너 자름 → Kp 너무 작음 → 2/0 로 ↑
 *   · 화면에 [CLAMP] 자주 뜨면 → 보정이 모터 한계까지 침 = Kp 큼 or 속도 과함
 *   · 직진 매끄럽고 곡선도 부드럽게 = 적정. 그 후 속도(3/4) 올리며 재확인
 *
 * ※ 센서 보정값은 베이스와 반드시 동일하게 유지!
 ****************************************************************/

//===== 핀 (Jellibi_AGV2_강의안.pdf) =====
#define pinLTLeft   A6
#define pinLTRight  A7
#define pinBuzzer   3
#define pinPWM1     5
#define pinPWM2     6
#define pinDIR1     7
#define pinDIR2     8

#define FORWARD  0
#define BACKWARD 1

//===== 센서 정규화 (베이스와 동일하게!) =====
#define LEFT_WHITE   411
#define LEFT_BLACK   926
#define RIGHT_WHITE  395
#define RIGHT_BLACK  936

//===== 조정 대상 값 (초기값 = 현재 베이스 값) =====
float Kp          = 0.02;
int   basePower   = 100;
int   minPower    = 75;
int   maxPower    = 145;
float Power1RatioF = 1.00;
float Power2RatioF = 1.05;

bool  running = false;
unsigned long lastDbg = 0;

//===== drive (베이스와 동일) =====
void drive(int dir1, int power1, int dir2, int power2) {
  boolean hl1, hl2;
  int p1, p2;
  if (dir1 == FORWARD) { hl1 = HIGH; p1 = power1 * Power1RatioF; }
  else                 { hl1 = LOW;  p1 = power1; }
  if (dir2 == FORWARD) { hl2 = LOW;  p2 = power2 * Power2RatioF; }
  else                 { hl2 = HIGH; p2 = power2; }
  p1 = constrain(p1, 0, 255);
  p2 = constrain(p2, 0, 255);
  digitalWrite(pinDIR1, hl1); analogWrite(pinPWM1, p1);
  digitalWrite(pinDIR2, hl2); analogWrite(pinPWM2, p2);
}
void Stop() { analogWrite(pinPWM1, 0); analogWrite(pinPWM2, 0); }

int normalize(int raw, int w, int b) {
  if (b == w) return 0;
  long n = (long)(raw - w) * 1000L / (b - w);
  if (n < 0)    n = 0;
  if (n > 1000) n = 1000;
  return (int)n;
}

//===== 라인트레이싱 한 스텝 (베이스 LineTracePControl 과 동일 + 텔레메트리) =====
void lineTraceStep() {
  int l = normalize(analogRead(pinLTLeft),  LEFT_WHITE,  LEFT_BLACK);
  int r = normalize(analogRead(pinLTRight), RIGHT_WHITE, RIGHT_BLACK);
  int error = l - r;
  int correction = (int)(error * Kp);
  int lp = constrain(basePower - correction, minPower, maxPower);
  int rp = constrain(basePower + correction, minPower, maxPower);
  drive(FORWARD, lp, FORWARD, rp);

  if (millis() - lastDbg >= 150) {
    lastDbg = millis();
    Serial.print(F("L="));    Serial.print(l);
    Serial.print(F(" R="));   Serial.print(r);
    Serial.print(F(" err=")); Serial.print(error);
    Serial.print(F(" corr="));Serial.print(correction);
    Serial.print(F(" lp="));  Serial.print(lp);
    Serial.print(F(" rp="));  Serial.print(rp);
    if (lp == minPower || lp == maxPower || rp == minPower || rp == maxPower)
      Serial.print(F("  [CLAMP]"));
    Serial.println();
  }
}

void printSettings() {
  Serial.println(F("---- 현재 설정 (베이스에 복사) ----"));
  Serial.print(F("  Kp        = ")); Serial.println(Kp, 3);
  Serial.print(F("  basePower = ")); Serial.println(basePower);
  Serial.print(F("  minPower  = ")); Serial.println(minPower);
  Serial.print(F("  maxPower  = ")); Serial.println(maxPower);
  Serial.println(F("-----------------------------------"));
}

void setup() {
  Serial.begin(9600);
  pinMode(pinLTLeft, INPUT);
  pinMode(pinLTRight, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinDIR1, OUTPUT); pinMode(pinPWM1, OUTPUT);
  pinMode(pinDIR2, OUTPUT); pinMode(pinPWM2, OUTPUT);
  Stop();

  Serial.println(F("=== Line Trace Tuner ==="));
  Serial.println(F("g시작 s정지 | 1/2 Kp+-0.005  9/0 Kp+-0.001"));
  Serial.println(F("3/4속도  5/6 max  7/8 min  p출력"));
  printSettings();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'g': running = true;  tone(pinBuzzer, 880, 80);
                Serial.println(F(">> RUN")); break;
      case 's': running = false; Stop();
                Serial.println(F(">> STOP")); break;
      case '1': Kp -= 0.005; if (Kp < 0) Kp = 0; printSettings(); break;
      case '2': Kp += 0.005; printSettings(); break;
      case '9': Kp -= 0.001; if (Kp < 0) Kp = 0; printSettings(); break;
      case '0': Kp += 0.001; printSettings(); break;
      case '3': basePower -= 5; printSettings(); break;
      case '4': basePower += 5; printSettings(); break;
      case '5': maxPower  -= 5; printSettings(); break;
      case '6': maxPower  += 5; printSettings(); break;
      case '7': minPower  -= 5; printSettings(); break;
      case '8': minPower  += 5; printSettings(); break;
      case 'p': printSettings(); break;
      default: break;
    }
  }

  if (running) lineTraceStep();
  else         Stop();
}
