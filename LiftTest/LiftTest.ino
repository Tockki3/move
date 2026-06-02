/****************************************************************
 * Jellibi AGV - 리프트(서보) 단독 테스트 (목차 9단계)
 *   · D9 서보를 시리얼로 각도 조정 → 상차(올림)/하차(내림) 각도 찾기
 *
 * ※ 업로드 시 서보가 현재 각도(90도)로 이동합니다.
 *    리프트가 부딪히지 않게 시작 전 손으로 받칠 준비!
 *    (구조상 가동범위가 좁으면 START_ANGLE 을 안전한 값으로 바꾸세요)
 *
 * [사용법]
 *   1. 업로드 → 시리얼 모니터(9600)
 *   2. 1/2(±5°), 3/4(±1°) 로 각도 조정하며 리프트 움직임 확인
 *   3. 내림 위치에서 'd', 올림 위치에서 'u' 로 각도 저장
 *   4. 'w'(내림)·'e'(올림) 로 두 위치 왕복 테스트
 *   5. 출력된 downAngle/upAngle 을 베이스 liftDown()/liftUp() 에 입력
 *
 * [명령] 1/2 ±5°  3/4 ±1°  | d=내림저장 u=올림저장 | w내림 e올림 p출력
 ****************************************************************/
#include <Servo.h>

#define pinLiftServo 9
#define START_ANGLE  90   // 시작 각도 (가동범위 좁으면 안전값으로 수정)

Servo lift;
int angle     = START_ANGLE;  // 현재 각도
int downAngle = START_ANGLE;   // 내림(하차)
int upAngle   = START_ANGLE;   // 올림(상차)

void printState() {
  Serial.print(F("angle=")); Serial.print(angle);
  Serial.print(F("   | downAngle(내림)=")); Serial.print(downAngle);
  Serial.print(F("  upAngle(올림)="));      Serial.println(upAngle);
}

void setup() {
  Serial.begin(9600);
  lift.attach(pinLiftServo);
  lift.write(angle);

  Serial.println(F("=== Lift(Servo) Test ==="));
  Serial.println(F("1/2 ±5  3/4 ±1 | d=내림저장 u=올림저장 | w내림 e올림 p출력"));
  printState();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case '1': angle -= 5; break;
    case '2': angle += 5; break;
    case '3': angle -= 1; break;
    case '4': angle += 1; break;
    case 'd': downAngle = angle; Serial.println(F(">> 내림(하차) 저장")); break;
    case 'u': upAngle   = angle; Serial.println(F(">> 올림(상차) 저장")); break;
    case 'w': angle = downAngle; break;   // 내림 위치로
    case 'e': angle = upAngle;   break;   // 올림 위치로
    case 'p': printState(); return;
    default: return;                      // 개행 등 무시
  }

  angle = constrain(angle, 0, 180);
  lift.write(angle);
  printState();
}
