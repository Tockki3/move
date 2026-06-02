/****************************************************************
 * Jellibi AGV - 모터/회전 보정용 임시 코드 (Motor & Turn Tuner)
 *   · 시리얼 모니터(9600)에서 키를 보내 동작 실행 + 값 조정
 *   · drive() 로직이 베이스와 동일 → 찾은 값 그대로 복사
 *
 * [명령] (시리얼 모니터 입력창에 글자 전송)
 *   w 전진(직진 테스트)   x 후진      s 즉시 정지
 *   a 좌회전(90도 확인)   d 우회전(90도 확인)   h 180도 회전
 *   1/2 : 모터파워 -5 / +5
 *   3/4 : 우모터보정 Power2RatioF -0.01 / +0.01   ← 직진 보정 핵심
 *   5/6 : 90도 회전시간 -10 / +10 ms
 *   7/8 : 180도 회전시간 -10 / +10 ms
 *   9/0 : 회전파워 -5 / +5
 *   p   : 현재 설정값 출력
 *
 * [보정 순서]
 *   ① 모터파워: w 전진하며 1/2 로 적당한 속도 (너무 느려 안 가면 ↑)
 *   ② 직진보정: w 전진 → 오른쪽으로 휘면 4(↑), 왼쪽으로 휘면 3(↓) → 직진될 때까지
 *   ③ 90도:    a / d → 덜 돌면 6(시간↑), 더 돌면 5(시간↓)
 *   ④ 180도:   h    → 덜 돌면 8(↑), 더 돌면 7(↓)
 *   ⑤ p 로 최종값 확인 → JellibiAGV_Base.ino 에 입력
 ****************************************************************/

//===== 핀 (Jellibi_AGV2_강의안.pdf) =====
#define pinBuzzer  3
#define pinPWM1    5   // 좌 모터 속도
#define pinPWM2    6   // 우 모터 속도
#define pinDIR1    7   // 좌 모터 방향
#define pinDIR2    8   // 우 모터 방향

#define FORWARD  0
#define BACKWARD 1

//===== 조정 대상 값 (초기값 = 현재 베이스 값) =====
int   motorPower   = 100;    // basePower
float Power1RatioF = 1.00;   // 좌 모터 보정
float Power2RatioF = 1.05;   // 우 모터 보정 (직진 보정 주 노브)
int   turnPower    = 90;     // turnPower
int   turnTime90   = 420;    // leftTurnTime / rightTurnTime
int   turnTime180  = 840;    // halfTurnTime

int   forwardTestTime = 1500; // 전/후진 테스트 지속(ms)

//===== drive (베이스와 동일 로직) =====
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
void beep(int f, int t) { tone(pinBuzzer, f, t); }

void printSettings() {
  Serial.println(F("---- 현재 설정 (베이스에 복사) ----"));
  Serial.print(F("  basePower    = ")); Serial.println(motorPower);
  Serial.print(F("  Power1RatioF = ")); Serial.println(Power1RatioF);
  Serial.print(F("  Power2RatioF = ")); Serial.println(Power2RatioF);
  Serial.print(F("  turnPower    = ")); Serial.println(turnPower);
  Serial.print(F("  leftTurnTime = rightTurnTime = ")); Serial.println(turnTime90);
  Serial.print(F("  halfTurnTime = ")); Serial.println(turnTime180);
  Serial.println(F("-----------------------------------"));
}

void setup() {
  Serial.begin(9600);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinDIR1, OUTPUT); pinMode(pinPWM1, OUTPUT);
  pinMode(pinDIR2, OUTPUT); pinMode(pinPWM2, OUTPUT);
  Stop();

  Serial.println(F("=== Motor & Turn Tuner ==="));
  Serial.println(F("w전진 x후진 s정지 | a좌90 d우90 h180"));
  Serial.println(F("1/2파워  3/4직진보정  5/6 90도시간  7/8 180도시간  9/0회전파워  p출력"));
  printSettings();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    // --- 동작 ---
    case 'w': Serial.println(F(">> 전진")); beep(880, 50);
              drive(FORWARD, motorPower, FORWARD, motorPower);
              delay(forwardTestTime); Stop(); break;
    case 'x': Serial.println(F(">> 후진")); beep(880, 50);
              drive(BACKWARD, motorPower, BACKWARD, motorPower);
              delay(forwardTestTime); Stop(); break;
    case 's': Stop(); Serial.println(F(">> 정지")); break;
    case 'a': Serial.println(F(">> 좌회전 90")); beep(660, 50);
              drive(BACKWARD, turnPower, FORWARD, turnPower);
              delay(turnTime90); Stop(); break;
    case 'd': Serial.println(F(">> 우회전 90")); beep(660, 50);
              drive(FORWARD, turnPower, BACKWARD, turnPower);
              delay(turnTime90); Stop(); break;
    case 'h': Serial.println(F(">> 180 회전")); beep(440, 50);
              drive(BACKWARD, turnPower, FORWARD, turnPower);
              delay(turnTime180); Stop(); break;

    // --- 값 조정 ---
    case '1': motorPower   -= 5;    printSettings(); break;
    case '2': motorPower   += 5;    printSettings(); break;
    case '3': Power2RatioF -= 0.01; printSettings(); break;
    case '4': Power2RatioF += 0.01; printSettings(); break;
    case '5': turnTime90   -= 10;   printSettings(); break;
    case '6': turnTime90   += 10;   printSettings(); break;
    case '7': turnTime180  -= 10;   printSettings(); break;
    case '8': turnTime180  += 10;   printSettings(); break;
    case '9': turnPower    -= 5;    printSettings(); break;
    case '0': turnPower    += 5;    printSettings(); break;
    case 'p': printSettings(); break;

    default: break;  // 개행문자 등 무시
  }
}
