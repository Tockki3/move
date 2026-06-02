/****************************************************************
 * Jellibi AGV - 센서 보정용 임시 코드 (Calibration Tool)
 *   · 모터 동작 없음 (안전). 시리얼 모니터 9600 baud.
 *   · 바닥 라인센서(A6/A7) WHITE/BLACK 보정 + 전방 IR(A0/A1/A2) 확인
 *
 * [사용법]
 *  1. 업로드 → 시리얼 모니터를 9600 baud로 열기
 *  2. 바닥센서(A6/A7)를 "흰 바닥"과 "검은 라인/교차점" 위로 천천히 왕복
 *  3. 출력의 min/max 를 읽어 JellibiAGV_Base.ino 상단에 그대로 입력:
 *        LEFT_WHITE  = L의 min   (흰 바닥일 때 값)
 *        LEFT_BLACK  = L의 max   (검정 위일 때 값)
 *        RIGHT_WHITE = R의 min
 *        RIGHT_BLACK = R의 max
 *  4. 보정 후 norm 이 흰 바닥=0근처, 교차점=1000근처면 성공.
 *     ★ 교차점에서 L·R "둘 다" norm 700을 넘으면 카운트가 정상 동작합니다.
 *  5. 전방 IR: 앞에 장애물을 댔다/뗐다 하며 A0·A1·A2 값 변화를 확인
 *        → obstacleThreshold 는 "장애물 있을 때"와 "없을 때" 값 사이로 결정
 *  6. 시리얼 모니터 입력창에 아무 글자나 보내면(Enter) min/max 리셋
 ****************************************************************/

//===== 핀 (Jellibi_AGV2_강의안.pdf 기준) =====
#define pinLTLeft   A6   // 바닥 왼쪽 IR (라인)
#define pinLTRight  A7   // 바닥 오른쪽 IR (라인)
#define pinFrontC   A0   // 전면 가운데 IR
#define pinFrontL   A1   // 전면 왼쪽 IR (현재 장애물 검출에 사용)
#define pinFrontR   A2   // 전면 오른쪽 IR

//===== 현재 보정 기준 (norm 계산 확인용 — 보정 끝나면 이 값을 갱신) =====
int LEFT_WHITE  = 362,  LEFT_BLACK  = 919;
int RIGHT_WHITE = 276,  RIGHT_BLACK = 900;

//===== min/max 추적 =====
int lMin, lMax, rMin, rMax;

void resetMinMax() {
  lMin = 1023; lMax = 0;
  rMin = 1023; rMax = 0;
}

int normalize(int raw, int w, int b) {
  if (b == w) return 0;                 // 0 나눗셈 방지
  long n = (long)(raw - w) * 1000L / (b - w);
  if (n < 0)    n = 0;
  if (n > 1000) n = 1000;
  return (int)n;
}

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinLTLeft,  INPUT);
  pinMode(pinLTRight, INPUT);
  pinMode(pinFrontC,  INPUT);
  pinMode(pinFrontL,  INPUT);
  pinMode(pinFrontR,  INPUT);

  resetMinMax();

  Serial.println(F("=== Sensor Calibration ==="));
  Serial.println(F("바닥센서를 흰/검 위로 천천히 왕복하세요."));
  Serial.println(F("min = 흰색 기준(WHITE),  max = 검정 기준(BLACK)"));
  Serial.println(F("아무 글자나 전송하면 min/max 리셋"));
  Serial.println();
}

void loop() {
  int rawL = analogRead(pinLTLeft);
  int rawR = analogRead(pinLTRight);

  if (rawL < lMin) lMin = rawL;
  if (rawL > lMax) lMax = rawL;
  if (rawR < rMin) rMin = rawR;
  if (rawR > rMax) rMax = rawR;

  // 시리얼로 아무 글자나 받으면 min/max 리셋
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    resetMinMax();
    Serial.println(F(">> min/max 리셋"));
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    int nL = normalize(rawL, LEFT_WHITE, LEFT_BLACK);
    int nR = normalize(rawR, RIGHT_WHITE, RIGHT_BLACK);

    int fC = analogRead(pinFrontC);
    int fL = analogRead(pinFrontL);
    int fR = analogRead(pinFrontR);

    // 바닥 왼쪽
    Serial.print(F("L raw=")); Serial.print(rawL);
    Serial.print(F(" min="));  Serial.print(lMin);
    Serial.print(F(" max="));  Serial.print(lMax);
    Serial.print(F(" norm=")); Serial.print(nL);

    // 바닥 오른쪽
    Serial.print(F("  |  R raw=")); Serial.print(rawR);
    Serial.print(F(" min="));  Serial.print(rMin);
    Serial.print(F(" max="));  Serial.print(rMax);
    Serial.print(F(" norm=")); Serial.print(nR);

    // 전방 IR
    Serial.print(F("  ||  Front  A0=")); Serial.print(fC);
    Serial.print(F(" A1="));  Serial.print(fL);
    Serial.print(F(" A2="));  Serial.print(fR);

    // 교차점 판정 미리보기 (둘 다 700↑ 이면 OK)
    if (nL > 700 && nR > 700) Serial.print(F("   <== CROSS!"));

    Serial.println();
  }
}
