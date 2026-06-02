/****************************************************************
 * Jellibi AGV - 통합 베이스 (Integrated Base)
 * 항법: 좌표 MoveTo 척추 + 계층형 하이브리드
 * 설계: 설계결정_항법아키텍처.md  /  핀맵: Jellibi_AGV2_강의안.pdf
 *
 * [구현됨]
 *   드라이버/HAL · P제어 라인트레이싱 · 라인카운터 · DoLineTrace(n)+중앙정렬
 *   시간기반 회전(heading 추적) · 좌표 MoveTo(row,col) · 격자 경계(inBounds)
 *   전방 장애물 검출(검출 시 안전 정지)
 *
 * [스텁 - 로드맵대로 채울 부분: "TODO:" 검색]
 *   RFID UID(8·12) · 리프트 서보(9·10) · 경계 안전 회피 모션(17·18) · 미션 시퀀서(19)
 *
 * [사용법]
 *   - 맵: START/START_DIR/HUB, cityTable UID를 대회 맵·공지에 맞게 수정
 *   - 실행 모드: RUN_MODE 로 전환 (기본 = 검증된 targetList 주행)
 ****************************************************************/

//==============================================================
// 1. 핀 맵 (Jellibi_AGV2_강의안.pdf 기준)
//==============================================================
#define pinLTLeft    A6   // 바닥 왼쪽 IR (라인)
#define pinLTRight   A7   // 바닥 오른쪽 IR (라인)
#define pinFrontC    A0   // 전면 가운데 IR
#define pinFrontL    A1   // 전면 왼쪽 IR  (장애물 검출에 사용 - 기존 보정값 기준)
#define pinFrontR    A2   // 전면 오른쪽 IR
#define pinButton    A3   // 측면 사용자 버튼

#define pinBuzzer    3    // 부저
#define pinPWM1      5    // 좌 모터 속도(PWM)
#define pinPWM2      6    // 우 모터 속도(PWM)
#define pinDIR1      7    // 좌 모터 방향
#define pinDIR2      8    // 우 모터 방향

// RFID(SPI 예약): D2(SS) D4(RST) D11(MOSI) D12(MISO) D13(SCK)
#define pinRfidSS    2
#define pinRfidRST   4
// 리프트 서보 예약
#define pinLiftServo 9

//==============================================================
// 2. 통일 튜닝 상수 (9단계 keeper 채택)
//==============================================================
// 센서 정규화 기준 (실측 — 교체한 로봇 기준)
#define LEFT_WHITE   411
#define LEFT_BLACK   926
#define RIGHT_WHITE  395
#define RIGHT_BLACK  936

// 교차점 임계값
#define LINE_COUNT_THRESHOLD 700
#define LINE_LEAVE_THRESHOLD 450

// 방향 의미값(모터)
#define FORWARD  0
#define BACKWARD 1

// 주행
int   basePower = 100;
int   minPower  = 75;
int   maxPower  = 145;
float Kp        = 0.02;
float Power1RatioF = 1.00;
float Power2RatioF = 1.05;

// 회전
int turnPower               = 90;
int forwardIntoCrossTime    = 50;
int forwardAfterTurnTime    = 0;
int leftTurnTime            = 420;
int rightTurnTime           = 420;
int halfTurnTime            = 840;
int forwardAfterHalfTurnTime = 60;

// 도착 교차점 중앙정렬
int arriveCrossForwardTime = 60;

// 회전 직후 교차점 무시
int turnIgnoreTime = 400;

// 장애물 (전방 IR: 값이 기준보다 작으면 장애물 / A1 기준 보정값)
// ※ 이 로봇 전방센서 미보정 → 오작동으로 잠시 OFF. 보정 후 1로 복구.
#define USE_OBSTACLE 0
int obstacleThreshold = 980;

//==============================================================
// 3. 좌표 / 방향 / 격자 맵 모델
//==============================================================
struct Point { int row; int col; };

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// --- 좌표 모델 (WCRC: 8×8 격자 + 사방 격자 밖 T스텁) ---
//   행: row0=창고줄(위), row1~8=격자 A~H, row9=도시줄(아래)
//   열: col0=좌측출발, col1~8=격자 열1~8, col9=우측출발
//   격자 내부(행1~8 × 열1~8)만 사방 연속. 스텁줄은 게이트(한 점)로만 연결.
#define ROW_MIN 0
#define ROW_MAX 9
#define COL_MIN 0
#define COL_MAX 9
#define GRID_ROW_MIN 1   // 행 A
#define GRID_ROW_MAX 8   // 행 H
#define GRID_COL_MIN 1   // 열 1
#define GRID_COL_MAX 8   // 열 8

// --- 창고(row0, 열1~8) / 도시(row9, 열1~8) ---
enum City { SEOUL, INCHEON, SEJONG, DAEJEON, DAEGU, GWANGJU, BUSAN, JEJU, CITY_COUNT };
Point cityPos[CITY_COUNT] = {       // 서울~제주 = 열1~8
  {9,1},{9,2},{9,3},{9,4},{9,5},{9,6},{9,7},{9,8}
};
Point warehousePos[8] = {           // 창고1~8 = 열1~8
  {0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7},{0,8}
};

// --- 출발지 / 허브 창고 (※ 대회 시작 시 공지된 값으로 수정) ---
// 출발지는 좌측(열0)·우측(열9) 또는 창고/도시 스텁 어디든 가능
Point     START     = {1, 0};    // TODO: 공지된 출발 (예: 1A 왼쪽 = (1,0))
Direction START_DIR = EAST;      // TODO: 출발 향(좌=EAST, 우=WEST, 창고=SOUTH, 도시=NORTH)
Point     HUB       = {0, 1};    // TODO: 상차 허브 창고 (예: 창고1 = (0,1))

Point     robotPos = {1, 0};
Direction robotDir = EAST;

bool inBounds(Point p) {
  return (p.row >= ROW_MIN && p.row <= ROW_MAX &&
          p.col >= COL_MIN && p.col <= COL_MAX);
}

// 방향 회전 헬퍼
Direction leftOf (Direction d) { return (Direction)((d + 3) % 4); }
Direction rightOf(Direction d) { return (Direction)((d + 1) % 4); }

// 전방 선언 (정의보다 먼저 호출되는 함수 — 자동 프로토타입 의존 제거)
bool CheckObstacle();
void handleObstacle();
void FinishAndStartNextTarget();

// p에서 방향 d로 n칸 이동한 좌표
Point stepCell(Point p, Direction d, int n) {
  Point r = p;
  if      (d == NORTH) r.row -= n;
  else if (d == SOUTH) r.row += n;
  else if (d == EAST)  r.col += n;
  else                 r.col -= n;
  return r;
}

//==============================================================
// 4. 전역 상태 변수
//==============================================================
// 라인 카운터
int  lineCounter = 0;
bool lineLock = false;
unsigned long lastLineCountTime = 0;
int  lineCountIgnoreTime  = 320;
int  leaveConfirmNeedCount = 2;
int  leaveConfirmCount = 0;
int  crossConfirmNeedCount = 2;
unsigned long ignoreCrossUntil = 0;

// DoLineTrace
int  targetLineCount = 0;
bool isLineTraceFinished = false;

// MoveTo 상태기계 (0 확인,1 row준비,2 row이동,3 col준비,4 col이동,5 도착)
int   moveState = 0;
Point currentTarget;

// 테스트 목적지 (스텁 라우팅 검증) — START=(1,0) 좌측출발 가정
// 창고1(0,1) → 제주(9,8) → 창고4(0,4)
Point targetList[] = { {0,1}, {9,8}, {0,4} };
int   targetCount  = 3;

// 스텁 안전 라우팅: 목적지까지 웨이포인트 (세로→가로→세로, 최대 4점)
Point routeWP[4];
int   routeLen = 0;
int   routeIdx = 0;
int   targetIndex  = 0;
bool  allTargetsFinished = false;

// 장애물 안전 정지 래치
bool obstacleHalt = false;

// 버튼 출발/정지 (A3)
bool running = false;   // 버튼으로 토글 (false = 출발 대기 / 일시정지)
bool started = false;   // 시퀀스 최초 시작 여부

// 비차단 부저
bool isBeepPlaying = false;
unsigned long beepEndTime = 0;

//==============================================================
// 5. 비차단 부저
//==============================================================
void BeepNonBlocking(int freq, int timeMs) {
  tone(pinBuzzer, freq, timeMs);
  isBeepPlaying = true;
  beepEndTime = millis() + timeMs;
}
void UpdateBuzzer() {
  if (isBeepPlaying && millis() >= beepEndTime) {
    noTone(pinBuzzer);
    isBeepPlaying = false;
  }
}

//==============================================================
// 6. 모터
//==============================================================
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
void Forward(int power)        { drive(FORWARD, power, FORWARD, power); }
void PivotTurnLeft(int power)  { drive(BACKWARD, power, FORWARD, power); }
void PivotTurnRight(int power) { drive(FORWARD, power, BACKWARD, power); }
void Stop() { analogWrite(pinPWM1, 0); analogWrite(pinPWM2, 0); }

//==============================================================
// 7. 센서
//==============================================================
int normalizeSensor(int rawValue, int whiteValue, int blackValue) {
  long n = (long)(rawValue - whiteValue) * 1000L;
  n = n / (blackValue - whiteValue);
  n = constrain(n, 0, 1000);
  return (int)n;
}
void ReadNormalizedSensors(int &leftNorm, int &rightNorm) {
  leftNorm  = normalizeSensor(analogRead(pinLTLeft),  LEFT_WHITE,  LEFT_BLACK);
  rightNorm = normalizeSensor(analogRead(pinLTRight), RIGHT_WHITE, RIGHT_BLACK);
}

//==============================================================
// 8. P제어 라인트레이싱
//==============================================================
void LineTracePControl() {
  int l, r;
  ReadNormalizedSensors(l, r);
  int error = l - r;
  int correction = (int)(error * Kp);
  int lp = constrain(basePower - correction, minPower, maxPower);
  int rp = constrain(basePower + correction, minPower, maxPower);
  drive(FORWARD, lp, FORWARD, rp);
}

//==============================================================
// 9. 라인 카운터
//==============================================================
bool IsCrossLineStable() {
  int c = 0;
  for (int i = 0; i < crossConfirmNeedCount; i++) {
    int l, r; ReadNormalizedSensors(l, r);
    if (l > LINE_COUNT_THRESHOLD && r > LINE_COUNT_THRESHOLD) c++;
    delay(1);
  }
  return (c >= crossConfirmNeedCount);
}
void ResetLineCounter() {
  lineCounter = 0; lineLock = false;
  lastLineCountTime = 0; leaveConfirmCount = 0;
}
void StartIgnoreCrossAfterTurn() {
  ignoreCrossUntil = millis() + turnIgnoreTime;
  ResetLineCounter();
}
void UpdateLineCounter() {
  if (millis() < ignoreCrossUntil) return;
  unsigned long now = millis();
  int l, r; ReadNormalizedSensors(l, r);
  bool crossNow = (l > LINE_COUNT_THRESHOLD && r > LINE_COUNT_THRESHOLD);
  bool leftCross = (l < LINE_LEAVE_THRESHOLD && r < LINE_LEAVE_THRESHOLD);

  if (crossNow) {
    leaveConfirmCount = 0;
    if (!lineLock && now - lastLineCountTime > lineCountIgnoreTime) {
      if (IsCrossLineStable()) {
        lineCounter++;
        lineLock = true;
        lastLineCountTime = now;
        BeepNonBlocking(1000, 30);
      }
    }
  }
  if (leftCross) {
    leaveConfirmCount++;
    if (leaveConfirmCount >= leaveConfirmNeedCount) {
      lineLock = false; leaveConfirmCount = 0;
    }
  } else {
    leaveConfirmCount = 0;
  }
}

//==============================================================
// 10. DoLineTrace(n) + 중앙정렬
//==============================================================
void StartDoLineTrace(int n) {
  targetLineCount = n;
  isLineTraceFinished = false;
  ResetLineCounter();
}
void FinishLineTraceWithCentering() {
  Stop(); delay(50);
  Forward(basePower); delay(arriveCrossForwardTime);  // 교차점 중앙으로 약간 더
  Stop(); delay(80);
  isLineTraceFinished = true;
  BeepNonBlocking(700, 100);
}
void RunDoLineTrace() {
  if (isLineTraceFinished) { Stop(); return; }

#if USE_OBSTACLE
  if (CheckObstacle()) { handleObstacle(); return; }
#endif

  UpdateLineCounter();
  if (lineCounter >= targetLineCount) { FinishLineTraceWithCentering(); return; }
  LineTracePControl();
}

//==============================================================
// 11. 회전 (heading 추적 포함)
//==============================================================
void UpdateDirectionLeft()  { robotDir = leftOf(robotDir); }
void UpdateDirectionRight() { robotDir = rightOf(robotDir); }
void UpdateDirectionHalf()  { robotDir = (Direction)((robotDir + 2) % 4); }

void TimeTurnLeft90() {
  Forward(basePower); delay(forwardIntoCrossTime);
  Stop(); delay(60);
  PivotTurnLeft(turnPower); delay(leftTurnTime);
  Stop(); delay(60);
  if (forwardAfterTurnTime > 0) { Forward(basePower); delay(forwardAfterTurnTime); Stop(); delay(60); }
  UpdateDirectionLeft();
  StartIgnoreCrossAfterTurn();
}
void TimeTurnRight90() {
  Forward(basePower); delay(forwardIntoCrossTime);
  Stop(); delay(60);
  PivotTurnRight(turnPower); delay(rightTurnTime);
  Stop(); delay(60);
  if (forwardAfterTurnTime > 0) { Forward(basePower); delay(forwardAfterTurnTime); Stop(); delay(60); }
  UpdateDirectionRight();
  StartIgnoreCrossAfterTurn();
}
void TimeTurnHalf() {
  Forward(basePower); delay(forwardIntoCrossTime);
  Stop(); delay(80);
  PivotTurnLeft(turnPower); delay(halfTurnTime);
  Stop(); delay(100);
  if (forwardAfterHalfTurnTime > 0) { Forward(basePower); delay(forwardAfterHalfTurnTime); Stop(); delay(80); }
  UpdateDirectionHalf();
  StartIgnoreCrossAfterTurn();
}
// 현재 방향에서 목표 방향으로 최소 회전
void TurnTo(Direction targetDir) {
  int diff = (targetDir - robotDir + 4) % 4;
  if      (diff == 1) TimeTurnRight90();
  else if (diff == 3) TimeTurnLeft90();
  else if (diff == 2) TimeTurnHalf();
  // diff == 0 : 회전 불필요
}

//==============================================================
// 12. 좌표 항법 MoveTo(row,col)
//==============================================================
void StartMoveTo(Point target) {
  currentTarget = target;
  isLineTraceFinished = false;
  moveState = 0;
  ResetLineCounter();
}
void UpdatePositionForward(int cells) {
  robotPos = stepCell(robotPos, robotDir, cells);
}

// 두 점이 같은가
bool samePoint(Point a, Point b) { return a.row == b.row && a.col == b.col; }

// 스텁(창고/도시/좌우출발)의 격자 진입점(게이트)
Point gateway(Point p) {
  Point g = p;
  if      (p.row == ROW_MIN) g.row = GRID_ROW_MIN;   // 창고(위)  → 아래로
  else if (p.row == ROW_MAX) g.row = GRID_ROW_MAX;   // 도시(아래)→ 위로
  else if (p.col == COL_MIN) g.col = GRID_COL_MIN;   // 좌측출발  → 오른쪽
  else if (p.col == COL_MAX) g.col = GRID_COL_MAX;   // 우측출발  → 왼쪽
  return g;                                          // 격자 내부면 그대로
}

// from→to 분해: [스텁 진입] → 격자 이동 → [스텁 탈출]
// 격자 내부는 사방 연속이라 MoveTo(행→열)가 한 번에 처리
int planRoute(Point from, Point to, Point wp[]) {
  int n = 0;
  wp[n++] = from;
  Point gA = gateway(from);
  Point gB = gateway(to);
  if (!samePoint(gA, from)) wp[n++] = gA;   // 스텁 진입(게이트)
  if (!samePoint(gB, gA))   wp[n++] = gB;   // 격자 내부 이동
  if (!samePoint(to, gB))   wp[n++] = to;   // 스텁 탈출
  return n;
}
void StartGoTo(Point dest) {
  routeLen = planRoute(robotPos, dest, routeWP);
  routeIdx = 1;                                // wp[0] = 현재 위치
  if (routeIdx < routeLen) StartMoveTo(routeWP[routeIdx]);
  else moveState = 5;                          // 이미 도착(같은 칸) → 다음으로
}
void RunMoveTo() {
  if (allTargetsFinished) { Stop(); return; }

  // 0. 목표 확인
  if (moveState == 0) {
    if      (robotPos.row != currentTarget.row) moveState = 1;
    else if (robotPos.col != currentTarget.col) moveState = 3;
    else                                        moveState = 5;
  }
  // 1. row 이동 준비
  else if (moveState == 1) {
    int d = currentTarget.row - robotPos.row;
    if (d < 0) { TurnTo(NORTH); StartDoLineTrace(abs(d)); }
    else       { TurnTo(SOUTH); StartDoLineTrace(abs(d)); }
    moveState = 2;
  }
  // 2. row 이동
  else if (moveState == 2) {
    RunDoLineTrace();
    if (isLineTraceFinished) {
      UpdatePositionForward(abs(currentTarget.row - robotPos.row));
      isLineTraceFinished = false; ResetLineCounter(); moveState = 0;
    }
  }
  // 3. col 이동 준비
  else if (moveState == 3) {
    int d = currentTarget.col - robotPos.col;
    if (d < 0) { TurnTo(WEST); StartDoLineTrace(abs(d)); }
    else       { TurnTo(EAST); StartDoLineTrace(abs(d)); }
    moveState = 4;
  }
  // 4. col 이동
  else if (moveState == 4) {
    RunDoLineTrace();
    if (isLineTraceFinished) {
      UpdatePositionForward(abs(currentTarget.col - robotPos.col));
      isLineTraceFinished = false; ResetLineCounter(); moveState = 0;
    }
  }
  // 5. (웨이포인트) 도착
  else if (moveState == 5) {
    Stop(); moveState = 0;
    routeIdx++;
    if (routeIdx < routeLen) StartMoveTo(routeWP[routeIdx]); // 다음 웨이포인트
    else                     FinishAndStartNextTarget();     // 최종 목적지 도착
  }
}

//==============================================================
// 13. 다중 목표 순차 주행 (기본 검증 모드)
//==============================================================
void StartCurrentTarget() {
  if (targetIndex >= targetCount) {
    allTargetsFinished = true; Stop(); BeepNonBlocking(1500, 500); return;
  }
  StartGoTo(targetList[targetIndex]);
}
void FinishAndStartNextTarget() {
  targetIndex++;
  delay(300);
  if (targetIndex >= targetCount) {
    allTargetsFinished = true; Stop(); BeepNonBlocking(1500, 500); return;
  }
  StartCurrentTarget();
}

//==============================================================
// 14. 장애물 검출 + 경계 안전 회피
//==============================================================
bool CheckObstacle() {
#if USE_OBSTACLE
  int hit = 0;
  for (int i = 0; i < 5; i++) {
    if (analogRead(pinFrontL) < obstacleThreshold) hit++;
    delay(2);
  }
  return (hit >= 5);
#else
  return false;
#endif
}

// 경계 안전 회피: 안쪽(inBounds) 방향을 고른다.
// ※ 실제 우회 "모션"은 맵 칸 간격에 맞춰 튜닝 필요 → 현재는 안전 정지(스텁).
void handleObstacle() {
  Stop();
  BeepNonBlocking(400, 300);

  Point sideRight = stepCell(robotPos, rightOf(robotDir), 1);
  Point sideLeft  = stepCell(robotPos, leftOf(robotDir),  1);

  int side = 0;                 // 0 없음, 1 우, 2 좌
  if      (inBounds(sideRight)) side = 1;
  else if (inBounds(sideLeft))  side = 2;

  // TODO(목차 17·18): 아래를 실제 우회 모션으로 구현
  //   runAvoidPath(side);                          // 우회 주행
  //   robotPos = stepCell(robotPos, robotDir, 2);  // 장애물 너머 칸으로 위치 갱신
  //   ResetLineCounter();                          // 복귀 후 카운터 초기화
  //   return;  // MoveTo가 남은 경로 재계산
  (void)side;

  // 회피 구현 전까지: 안전 정지 래치
  obstacleHalt = true;
}

//==============================================================
// 15. 모듈 스텁 (로드맵대로 채우기)
//==============================================================
// ---- RFID (목차 8·12) ----  D2(SS)/D4(RST)/SPI, MFRC522 권장
struct CityTag { byte uid[4]; Point pos; };
// 각 도시 태그 UID를 RfidTest로 읽어 채우세요 (좌표 = cityPos)
CityTag cityTable[CITY_COUNT] = {
  { {0,0,0,0}, {9,1} },  // SEOUL   서울
  { {0,0,0,0}, {9,2} },  // INCHEON 인천
  { {0,0,0,0}, {9,3} },  // SEJONG  세종
  { {0,0,0,0}, {9,4} },  // DAEJEON 대전
  { {0,0,0,0}, {9,5} },  // DAEGU   대구
  { {0,0,0,0}, {9,6} },  // GWANGJU 광주
  { {0,0,0,0}, {9,7} },  // BUSAN   부산
  { {0,0,0,0}, {9,8} },  // JEJU    제주
};
// 읽은 태그 {0x03,0x04,0x8C,0xE2} → 어느 도시인지 확인 후 해당 줄 uid에 입력
bool readUID(byte *uid) {
  // TODO: RFID 모듈에서 UID 읽기. 읽으면 true.
  return false;
}
Point lookupCity(byte *uid) {
  for (int i = 0; i < CITY_COUNT; i++) {
    bool eq = true;
    for (int b = 0; b < 4; b++) if (cityTable[i].uid[b] != uid[b]) eq = false;
    if (eq) return cityTable[i].pos;
  }
  return HUB; // 미매칭 시 기본값(허브 창고)
}

// ---- 리프트 (목차 9·10) ----  D9 서보
void liftUp()   { /* TODO: 서보 상승(상차) */ }
void liftDown() { /* TODO: 서보 하강(하차) */ }

//==============================================================
// 16. 미션 시퀀서 (목차 19) - 스텁
//==============================================================
enum MissionState { M_IDLE, M_READ_RFID, M_TO_WAREHOUSE, M_LOAD,
                    M_TO_CITY, M_UNLOAD, M_RETURN, M_DONE };
MissionState mState = M_IDLE;
Point missionDest;

void RunMission() {
  switch (mState) {
    case M_IDLE:
      mState = M_READ_RFID;
      break;
    case M_READ_RFID: {
      byte uid[4];
      if (readUID(uid)) { missionDest = lookupCity(uid); mState = M_TO_WAREHOUSE; }
      // TODO: 읽기 실패 시 재시도/대기 처리
      break;
    }
    case M_TO_WAREHOUSE:
      // TODO: StartGoTo(HUB) 후 RunMoveTo 완료까지 진행 → M_LOAD
      break;
    case M_LOAD:
      liftUp(); mState = M_TO_CITY;
      break;
    case M_TO_CITY:
      // TODO: StartMoveTo(missionDest) 진행 → 완료 시 M_UNLOAD
      break;
    case M_UNLOAD:
      liftDown(); mState = M_RETURN;
      break;
    case M_RETURN:
      // TODO: StartMoveTo(START) 진행 → 완료 시 M_DONE
      break;
    case M_DONE:
      Stop();
      break;
  }
}

//==============================================================
// 17. 실행 모드
//==============================================================
#define MODE_TEST_MOVE 0   // 검증된 targetList 좌표 주행 (기본)
#define MODE_MISSION   1   // RFID 미션 (스텁 완성 후 전환)
#define RUN_MODE MODE_TEST_MOVE

// 디버그 텔레메트리 (Serial Monitor 9600 baud). 문제 해결 후 0으로.
#define DEBUG 1
unsigned long lastDbgTime = 0;
void DebugPrint() {
#if DEBUG
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();
  int rawL = analogRead(pinLTLeft);
  int rawR = analogRead(pinLTRight);
  int l = normalizeSensor(rawL, LEFT_WHITE, LEFT_BLACK);
  int r = normalizeSensor(rawR, RIGHT_WHITE, RIGHT_BLACK);
  Serial.print("state="); Serial.print(moveState);
  Serial.print(" cnt=");  Serial.print(lineCounter);
  Serial.print("/");      Serial.print(targetLineCount);
  Serial.print(" | rawL="); Serial.print(rawL);
  Serial.print(" rawR=");   Serial.print(rawR);
  Serial.print(" | L=");    Serial.print(l);
  Serial.print(" R=");      Serial.print(r);
  Serial.print(" | front="); Serial.print(analogRead(pinFrontL));
  Serial.print(" | pos=(");  Serial.print(robotPos.row);
  Serial.print(",");        Serial.print(robotPos.col);
  Serial.print(") dir=");   Serial.println(robotDir);
#endif
}

//==============================================================
// 18. 버튼 출발/정지 (A3, 액티브 로우, INPUT 외부풀업)
//==============================================================
void handleStartButton() {
  if (digitalRead(pinButton) != 0) return;        // 안 눌림
  Stop();                                          // 눌리는 순간 안전 정지
  delay(20);                                       // 디바운스
  if (digitalRead(pinButton) != 0) return;         // 채터링 무시
  while (digitalRead(pinButton) == 0) delay(5);    // 손 뗄 때까지 대기

  running = !running;                              // 토글
  if (running) {
    if (!started) {                                // 최초 1회: 시퀀스 시작
      started = true;
      if (RUN_MODE == MODE_TEST_MOVE) StartCurrentTarget();
    }
    Serial.println(F(">> START"));
    BeepNonBlocking(880, 120);
  } else {                                         // 다시 누르면 정지(비상정지)
    Stop();
    Serial.println(F(">> STOP"));
    BeepNonBlocking(300, 200);
  }
}

//==============================================================
// 19. setup / loop
//==============================================================
void setup() {
  Serial.begin(9600);

  pinMode(pinLTLeft, INPUT);
  pinMode(pinLTRight, INPUT);
  pinMode(pinFrontL, INPUT);
  pinMode(pinButton, INPUT);

  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinDIR1, OUTPUT); pinMode(pinPWM1, OUTPUT);
  pinMode(pinDIR2, OUTPUT); pinMode(pinPWM2, OUTPUT);

  Stop();

  // 시작 자세 고정
  robotPos = START;
  robotDir = START_DIR;

  delay(300);
  BeepNonBlocking(523, 150);
  Serial.println(F("Jellibi AGV Base - 버튼(A3)을 누르면 출발"));
  // 버튼 출발: setup에서 자동 시작하지 않음 (loop의 handleStartButton)
}

void loop() {
  UpdateBuzzer();
  DebugPrint();
  handleStartButton();                    // 버튼으로 출발/정지 토글

  if (!running)     { Stop(); return; }   // 출발 대기 / 일시정지
  if (obstacleHalt) { Stop(); return; }   // 장애물 안전 정지 래치

  if (RUN_MODE == MODE_TEST_MOVE) RunMoveTo();
  else                            RunMission();
}
