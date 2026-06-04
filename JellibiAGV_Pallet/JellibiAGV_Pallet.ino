/****************************************************************
 * Jellibi AGV - 팔렛 RFID 목적지 미션
 *   스타트 RFID → 창고 이동 → [팔렛 RFID로 목적지 결정] → 적재
 *               → 그 도시로 이동 → 하차 → 창고 반복
 *   라이브러리: MFRC522, Servo(내장)
 ****************************************************************/
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

//==================== 핀 맵 ====================
#define pinLTLeft    A6   // 바닥 왼쪽 IR (라인)
#define pinLTRight   A7   // 바닥 오른쪽 IR (라인)
#define pinButton    A3   // 버튼(비상정지)
#define pinBuzzer    3
#define pinPWM1      5    // 좌 모터 속도
#define pinPWM2      6    // 우 모터 속도
#define pinDIR1      7    // 좌 모터 방향
#define pinDIR2      8    // 우 모터 방향
#define pinRfidSS    2
#define pinRfidRST   4
#define pinLiftServo 9
#define pinFrontC    A0   // 전면 가운데 IR (장애물 감지)

//==================== ★ 여기만 수정 ====================
int  useWarehouses[] = {1, 3, 5};  // ★ 진입 가능한 창고 번호 (1~8). 하차 후 가장 가까운 곳으로 복귀
#define USE_WH_N ((int)(sizeof(useWarehouses)/sizeof(useWarehouses[0])))
int  START_ROW    = 4;     // 출발 위치 행 (A=1 … H=8)
int  START_COL    = 0;     // 출발 위치 열: 0=좌측출발, 9=우측출발
byte START_CARD[4] = {0x74,0xDC,0x58,0x74};  // 시작 카드 UID (RfidTest로 확인)
int  LIFT_UP_ANGLE   = 150; // 적재(올림) 각도
int  LIFT_DOWN_ANGLE = 40;  // 하차(내림) 각도
int  actionDelayMs   = 1000;// 적재/하차 동작 시간(ms)
// ※ 목적지는 '팔렛 RFID'로 결정 → 아래 cityTable 에 UID 입력
// ※ 진입 불가 좌표는 아래 'blocked[]' 에서 추가
#define USE_OBSTACLE 1        // 전면 장애물 감지+우회 (끄려면 0)
int  obstacleThreshold = 980; // 전면 IR raw가 이 값보다 작으면 장애물 (SensorCalibration으로 보정)

//==================== 튜닝 상수 ====================
// --- 바닥센서 정규화 기준 (SensorCalibration으로 측정한 raw값) ---
#define LEFT_WHITE   415    // 좌센서 '흰 바닥' raw → 정규화 0
#define LEFT_BLACK   992    // 좌센서 '검정선' raw → 정규화 1000
#define RIGHT_WHITE  378    // 우센서 흰 바닥 raw
#define RIGHT_BLACK  959    // 우센서 검정선 raw
#define LINE_COUNT_THRESHOLD 700  // 교차점 판정: 두 센서 정규화값이 모두 이 값 이상이면 '교차점' (낮추면 더 민감)
#define LINE_LEAVE_THRESHOLD 450  // 교차점 이탈 판정: 두 센서 모두 이 값 미만이면 '벗어남'(다음 카운트 준비)

// --- 모터 방향 의미값 (바꾸지 말 것) ---
#define FORWARD  0          // 전진
#define BACKWARD 1          // 후진

// --- 주행 속도 / 라인트레이싱 ---
int   basePower = 110;      // 기본 주행 속도 PWM(0~255). ↑빠름·불안정 / ↓느림·안정
int   minPower  = 75;       // P제어 시 바퀴 최소 PWM (너무 낮으면 모터가 안 돎)
int   maxPower  = 145;      // P제어 시 바퀴 최대 PWM
float Kp           = 0.015; // 라인트레이싱 P게인. ↑민감(지그재그 떨림) / ↓둔함(곡선서 이탈)
float Power1RatioF = 0.99;  // 좌모터 출력 보정 배수 (직진이 한쪽으로 휘면 좌우 배수로 균형)
float Power2RatioF = 1.00;  // 우모터 출력 보정 배수

// --- 회전 (시간 기반: 배터리 잔량·바닥 마찰에 따라 재튜닝 필요) ---
int turnPower               = 100; // 제자리 회전 PWM (약하면 덜 돌고, 세면 더 돔)
int forwardIntoCrossTime    = 50;  // 회전 전, 교차점 중심까지 살짝 전진(ms)
int forwardAfterTurnTime    = 0;   // 90도 회전 후 추가 전진 보정(ms, 보통 0)
int forwardAfterHalfTurnTime= 60;  // 180도 회전 후 추가 전진 보정(ms)
int leftTurnTime            = 420; // 좌 90도 회전 시간(ms): 덜 돌면 ↑ / 더 돌면 ↓
int rightTurnTime           = 420; // 우 90도 회전 시간(ms)
int halfTurnTime            = 840; // 180도(반바퀴) 회전 시간(ms)
int arriveCrossForwardTime  = 60;  // 목표 교차점 도착 후 중앙정렬용 전진(ms)

//==================== 좌표계 ====================
//  행: 0=창고줄, 1~8=격자 A~H, 9=도시줄 / 열: 0=좌출발, 1~8=격자, 9=우출발
struct Point { int row, col; };
enum Direction { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
#define GRID_RMIN 1
#define GRID_RMAX 8
#define GRID_CMIN 1
#define GRID_CMAX 8

Point     HUB, START, missionDest;   // missionDest = 팔렛에서 읽은 목적지
Direction START_DIR;
Point     robotPos;
Direction robotDir;

Direction leftOf (Direction d){ return (Direction)((d+3)%4); }
Direction rightOf(Direction d){ return (Direction)((d+1)%4); }
Point stepCell(Point p, Direction d, int n){
  Point r = p;
  if      (d==NORTH) r.row -= n;
  else if (d==SOUTH) r.row += n;
  else if (d==EAST)  r.col += n;
  else               r.col -= n;
  return r;
}
bool samePoint(Point a, Point b){ return a.row==b.row && a.col==b.col; }

// ★ 진입 불가 좌표 (행1~8, 열1~8). 없으면 {0,0} 하나만. 예: { {3,3}, {5,2} }
Point blocked[] = { {0,0} };
#define BLOCKED_N ((int)(sizeof(blocked)/sizeof(blocked[0])))

// 런타임 차단(전면 장애물 감지로 추가되는 진입 불가 칸)
#define MAX_DYN_BLOCK 12
Point dynBlocked[MAX_DYN_BLOCK]; int dynCount = 0;
Point currentDest;                  // 현재 목적지(재경로용)
bool checkObstacle();               // 전방 선언
void handleObstacle();

// ★ 팔렛 UID → 목적지 도시  (각 팔렛을 RfidTest로 읽어 UID 입력)
//   도시 열: 1서울 2인천 3세종 4대전 5대구 6광주 7부산 8제주
struct CityTag { byte uid[4]; Point pos; };
CityTag cityTable[8] = {
  { {0,0,0,0}, {9,1} },  // 서울
  { {0,0,0,0}, {9,2} },  // 인천
  { {0,0,0,0}, {9,3} },  // 세종
  { {0,0,0,0}, {9,4} },  // 대전
  { {0,0,0,0}, {9,5} },  // 대구
  { {0,0,0,0}, {9,6} },  // 광주
  { {0,0,0,0}, {9,7} },  // 부산
  { {0,0,0,0}, {9,8} },  // 제주
};

//==================== RFID / 서보 ====================
MFRC522 mfrc522(pinRfidSS, pinRfidRST);
Servo   liftServo;

//==================== 상태 변수 ====================
int  lineCounter=0; bool lineLock=false;
unsigned long lastCountTime=0;
int  lineCountIgnore=320, leaveNeed=2, leaveCount=0, crossNeed=2;

int  targetCount=0; bool traceDone=false;     // DoLineTrace
int  moveState=0; Point moveTarget;            // MoveTo
Point wp[16]; int wpLen=0, wpIdx=0; bool navActive=false;  // 경로(BFS 우회 시 다점)
bool started=false, running=false;             // 실행
bool beepOn=false; unsigned long beepEnd=0;    // 부저

//==================== 부저(비차단) ====================
void beep(int f,int t){ tone(pinBuzzer,f,t); beepOn=true; beepEnd=millis()+t; }
void updateBeep(){ if(beepOn && millis()>=beepEnd){ noTone(pinBuzzer); beepOn=false; } }

//==================== 모터 ====================
void drive(int d1,int p1,int d2,int p2){
  boolean h1,h2; int a,b;
  if(d1==FORWARD){ h1=HIGH; a=p1*Power1RatioF; } else { h1=LOW; a=p1; }
  if(d2==FORWARD){ h2=LOW;  b=p2*Power2RatioF; } else { h2=HIGH; b=p2; }
  a=constrain(a,0,255); b=constrain(b,0,255);
  digitalWrite(pinDIR1,h1); analogWrite(pinPWM1,a);
  digitalWrite(pinDIR2,h2); analogWrite(pinPWM2,b);
}
void Forward(int p){ drive(FORWARD,p,FORWARD,p); }
void PivotLeft(int p){ drive(BACKWARD,p,FORWARD,p); }
void PivotRight(int p){ drive(FORWARD,p,BACKWARD,p); }
void Stop(){ analogWrite(pinPWM1,0); analogWrite(pinPWM2,0); }

//==================== 센서 ====================
int normalize(int raw,int w,int b){
  long n=(long)(raw-w)*1000L/(b-w);
  if(n<0)n=0; if(n>1000)n=1000; return (int)n;
}
void readSensors(int &l,int &r){
  l=normalize(analogRead(pinLTLeft),  LEFT_WHITE,  LEFT_BLACK);
  r=normalize(analogRead(pinLTRight), RIGHT_WHITE, RIGHT_BLACK);
}

//==================== 라인트레이싱 P제어 ====================
void lineTrace(){
  int l,r; readSensors(l,r);
  int corr=(int)((l-r)*Kp);
  drive(FORWARD, constrain(basePower-corr,minPower,maxPower),
        FORWARD, constrain(basePower+corr,minPower,maxPower));
}

//==================== 라인 카운터 ====================
bool crossStable(){
  int c=0;
  for(int i=0;i<crossNeed;i++){ int l,r; readSensors(l,r);
    if(l>LINE_COUNT_THRESHOLD && r>LINE_COUNT_THRESHOLD) c++; delay(1); }
  return c>=crossNeed;
}
void resetCounter(){ lineCounter=0; lineLock=false; lastCountTime=0; leaveCount=0; }
void ignoreAfterTurn(){ resetCounter(); lineLock=true; }
void updateCounter(){
  unsigned long now=millis();
  int l,r; readSensors(l,r);
  bool cross=(l>LINE_COUNT_THRESHOLD && r>LINE_COUNT_THRESHOLD);
  bool leave=(l<LINE_LEAVE_THRESHOLD && r<LINE_LEAVE_THRESHOLD);
  if(cross){
    leaveCount=0;
    if(!lineLock && now-lastCountTime>lineCountIgnore && crossStable()){
      lineCounter++; lineLock=true; lastCountTime=now; beep(1000,30);
    }
  }
  if(leave){ if(++leaveCount>=leaveNeed){ lineLock=false; leaveCount=0; } }
  else leaveCount=0;
}

//==================== DoLineTrace(n) ====================
void startTrace(int n){ targetCount=n; traceDone=false; resetCounter(); }
void finishTrace(){
  Stop(); delay(50); Forward(basePower); delay(arriveCrossForwardTime);
  Stop(); delay(80); traceDone=true; beep(700,100);
}
void runTrace(){
  if(traceDone){ Stop(); return; }
#if USE_OBSTACLE
  if(checkObstacle()){ handleObstacle(); return; }   // 전면 장애물 → 차단등록+후진+재경로
#endif
  updateCounter();
  if(lineCounter>=targetCount){ finishTrace(); return; }
  lineTrace();
}

//==================== 회전(heading 추적) ====================
void turnLeft90(){
  Forward(basePower); delay(forwardIntoCrossTime); Stop(); delay(60);
  PivotLeft(turnPower); delay(leftTurnTime); Stop(); delay(60);
  if(forwardAfterTurnTime>0){ Forward(basePower); delay(forwardAfterTurnTime); Stop(); delay(60); }
  robotDir=leftOf(robotDir); ignoreAfterTurn();
}
void turnRight90(){
  Forward(basePower); delay(forwardIntoCrossTime); Stop(); delay(60);
  PivotRight(turnPower); delay(rightTurnTime); Stop(); delay(60);
  if(forwardAfterTurnTime>0){ Forward(basePower); delay(forwardAfterTurnTime); Stop(); delay(60); }
  robotDir=rightOf(robotDir); ignoreAfterTurn();
}
void turnHalf(){
  Forward(basePower); delay(forwardIntoCrossTime); Stop(); delay(80);
  PivotLeft(turnPower); delay(halfTurnTime); Stop(); delay(100);
  if(forwardAfterHalfTurnTime>0){ Forward(basePower); delay(forwardAfterHalfTurnTime); Stop(); delay(80); }
  robotDir=(Direction)((robotDir+2)%4); ignoreAfterTurn();
}
void turnTo(Direction t){
  int diff=(t-robotDir+4)%4;
  if(diff==1) turnRight90(); else if(diff==3) turnLeft90(); else if(diff==2) turnHalf();
}

//==================== 좌표 항법 ====================
void startMove(Point t){ moveTarget=t; traceDone=false; moveState=0; resetCounter(); }
void posForward(int n){ robotPos=stepCell(robotPos,robotDir,n); }

Point gateway(Point p){
  Point g=p;
  if      (p.row==0) g.row=GRID_RMIN;
  else if (p.row==9) g.row=GRID_RMAX;
  else if (p.col==0) g.col=GRID_CMIN;
  else if (p.col==9) g.col=GRID_CMAX;
  return g;
}
int planRoute(Point from, Point to){
  int n=0; wp[n++]=from;
  Point gA=gateway(from), gB=gateway(to);
  if(!samePoint(gA,from)) wp[n++]=gA;
  if(!samePoint(gB,gA))   wp[n++]=gB;
  if(!samePoint(to,gB))   wp[n++]=to;
  return n;
}
//---- 진입 불가 좌표 우회 (BFS) ----
bool isInterior(Point p){ return p.row>=GRID_RMIN && p.row<=GRID_RMAX && p.col>=GRID_CMIN && p.col<=GRID_CMAX; }
bool isNode(Point p){
  if(isInterior(p)) return true;
  if(p.row==0 && p.col>=1 && p.col<=8) return true;
  if(p.row==9 && p.col>=1 && p.col<=8) return true;
  if(p.col==0 && p.row>=1 && p.row<=8) return true;
  if(p.col==9 && p.row>=1 && p.row<=8) return true;
  return false;
}
bool edgeOK(Point a, Point b){ return isNode(a)&&isNode(b)&&(isInterior(a)||isInterior(b)); }
bool isBlockedCell(Point p){
  for(int i=0;i<BLOCKED_N;i++) if(samePoint(blocked[i],p))    return true;  // 정적(설정) 차단
  for(int i=0;i<dynCount;i++)  if(samePoint(dynBlocked[i],p)) return true;  // 런타임(장애물) 차단
  return false;
}
void addBlocked(Point p){ if(!isBlockedCell(p) && dynCount<MAX_DYN_BLOCK) dynBlocked[dynCount++]=p; }
bool anyBlocks(){
  for(int i=0;i<BLOCKED_N;i++) if(isNode(blocked[i])) return true;
  return dynCount>0;
}

byte bfsPrev[100], bfsQ[100];
int planRouteBFS(Point from, Point to){
  for(int i=0;i<100;i++) bfsPrev[i]=255;
  int qh=0,qt=0; byte s=from.row*10+from.col, g=to.row*10+to.col;
  bfsQ[qt++]=s; bfsPrev[s]=s;
  const int DR[4]={-1,0,1,0}, DC[4]={0,1,0,-1};
  while(qh<qt){
    byte cur=bfsQ[qh++]; if(cur==g) break;
    Point cp={cur/10,cur%10};
    for(int d=0;d<4;d++){
      Point np={cp.row+DR[d], cp.col+DC[d]};
      if(np.row<0||np.row>9||np.col<0||np.col>9) continue;
      byte ni=np.row*10+np.col;
      if(bfsPrev[ni]!=255) continue;
      if(!edgeOK(cp,np) || isBlockedCell(np)) continue;
      bfsPrev[ni]=cur; bfsQ[qt++]=ni;
    }
  }
  if(bfsPrev[g]==255) return 0;
  int len=0; byte c=g; while(true){ bfsQ[len++]=c; if(c==s) break; c=bfsPrev[c]; }
  int n=0; wp[n++]=from;
  for(int i=len-2;i>=1;i--){
    int d1r=(bfsQ[i]/10)-(bfsQ[i+1]/10), d1c=(bfsQ[i]%10)-(bfsQ[i+1]%10);
    int d2r=(bfsQ[i-1]/10)-(bfsQ[i]/10), d2c=(bfsQ[i-1]%10)-(bfsQ[i]%10);
    if((d1r!=d2r||d1c!=d2c) && n<15){ Point t={bfsQ[i]/10,bfsQ[i]%10}; wp[n++]=t; }
  }
  wp[n++]=to; return n;
}

void goTo(Point dest){
  currentDest = dest;
  wpLen = anyBlocks() ? planRouteBFS(robotPos,dest) : planRoute(robotPos,dest);
  wpIdx=1; navActive=true;
  if(wpLen>=2 && wpIdx<wpLen) startMove(wp[wpIdx]);
  else { navActive=false; Stop(); beep(200,500); }   // 경로 없음
}
void runMoveTo(){
  if(!navActive){ Stop(); return; }
  if(moveState==0){
    if      (robotPos.row!=moveTarget.row) moveState=1;
    else if (robotPos.col!=moveTarget.col) moveState=3;
    else                                   moveState=5;
  }
  else if(moveState==1){
    int d=moveTarget.row-robotPos.row;
    turnTo(d<0?NORTH:SOUTH); startTrace(abs(d)); moveState=2;
  }
  else if(moveState==2){
    runTrace();
    if(traceDone){ posForward(abs(moveTarget.row-robotPos.row)); traceDone=false; resetCounter(); moveState=0; }
  }
  else if(moveState==3){
    int d=moveTarget.col-robotPos.col;
    turnTo(d<0?WEST:EAST); startTrace(abs(d)); moveState=4;
  }
  else if(moveState==4){
    runTrace();
    if(traceDone){ posForward(abs(moveTarget.col-robotPos.col)); traceDone=false; resetCounter(); moveState=0; }
  }
  else if(moveState==5){
    Stop(); moveState=0; wpIdx++;
    if(wpIdx<wpLen) startMove(wp[wpIdx]); else navActive=false;
  }
}

//==================== 전면 장애물 → 진입불가 등록 + 후진 + 재경로 ====================
bool checkObstacle(){
  int hit=0;
  for(int i=0;i<5;i++){ if(analogRead(pinFrontC) < obstacleThreshold) hit++; delay(2); }
  return hit>=5;
}
void handleObstacle(){
  Stop(); beep(400,200);
  // 1) 장애물 칸(현재 셀의 앞 칸)을 진입 불가로 등록
  Point cur = stepCell(robotPos, robotDir, lineCounter);
  Point obs = stepCell(cur, robotDir, 1);
  addBlocked(obs);
  Serial.print(F("OBSTACLE block(")); Serial.print(obs.row); Serial.print(','); Serial.print(obs.col); Serial.println(F(")"));
  // 2) 직전 교차점(cur)까지 후진
  unsigned long t0=millis();
  while(millis()-t0 < 3000){
    int l,r; readSensors(l,r);
    if(l>LINE_COUNT_THRESHOLD && r>LINE_COUNT_THRESHOLD) break;  // 교차점 도달
    drive(BACKWARD, basePower, BACKWARD, basePower);
    delay(5);
  }
  Stop(); delay(150);
  robotPos = cur;                 // 직전 교차점에 위치 (robotDir 그대로)
  // 3) 목적지로 경로 재설정 (막힌 칸 회피 BFS)
  goTo(currentDest);
}

//==================== RFID ====================
bool readUID(byte *uid){
  if(!mfrc522.PICC_IsNewCardPresent()) return false;
  if(!mfrc522.PICC_ReadCardSerial())  return false;
  for(byte i=0;i<4;i++) uid[i]=(i<mfrc522.uid.size)?mfrc522.uid.uidByte[i]:0;
  mfrc522.PICC_HaltA(); return true;
}
bool isStartCard(byte *uid){
  for(int i=0;i<4;i++) if(uid[i]!=START_CARD[i]) return false;
  return true;
}
Point lookupCity(byte *uid){             // 팔렛 UID → 도시 좌표
  for(int i=0;i<8;i++){
    bool eq=true;
    for(int b=0;b<4;b++) if(cityTable[i].uid[b]!=uid[b]) eq=false;
    if(eq) return cityTable[i].pos;
  }
  return HUB;  // 미등록 팔렛 → 배송 안 함(HUB로)
}

//==================== 리프트 ====================
void liftUp(){   liftServo.attach(pinLiftServo); liftServo.write(LIFT_UP_ANGLE);   delay(actionDelayMs); }                       // 적재(부착·유지)
void liftDown(){ liftServo.attach(pinLiftServo); liftServo.write(LIFT_DOWN_ANGLE); delay(actionDelayMs); liftServo.detach(); }  // 하차 후 분리

//==================== 미션: 창고→[팔렛읽기]→적재→도시→하차→(가까운 창고)반복 ====================
enum { M_TO_HUB, M_HUB_WAIT, M_READ_PALLET, M_LOAD, M_TO_CITY, M_CITY_WAIT, M_UNLOAD };
int mState = M_TO_HUB;

// 현재 위치에서 가장 가까운 '진입 가능' 창고 좌표 (창고는 모두 행0 → 열 거리로 판단)
Point nearestWarehouse(Point from){
  int bestCol = useWarehouses[0], bestDist = 9999;
  for(int i=0;i<USE_WH_N;i++){
    int d = abs(from.col - useWarehouses[i]);
    if(d < bestDist){ bestDist = d; bestCol = useWarehouses[i]; }
  }
  Point w = {0, bestCol};
  return w;
}

void runMission(){
  switch(mState){
    case M_TO_HUB:    HUB = nearestWarehouse(robotPos); goTo(HUB); mState=M_HUB_WAIT; break;  // 가장 가까운 창고로
    case M_HUB_WAIT:  if(!navActive) mState=M_READ_PALLET;  break;   // 도착 대기

    case M_READ_PALLET: {                                            // 팔렛 RFID → 목적지
      byte uid[4];
      if(readUID(uid)){
        missionDest = lookupCity(uid);
        Serial.print(F("PALLET:"));
        for(int i=0;i<4;i++){ Serial.print(' '); if(uid[i]<0x10) Serial.print('0'); Serial.print(uid[i],HEX); }
        Serial.print(F(" => dest("));Serial.print(missionDest.row);Serial.print(',');Serial.print(missionDest.col);Serial.println(')');
        beep(1200,100); mState=M_LOAD;
      }
      break;                                                         // 팔렛 없으면 대기
    }

    case M_LOAD:      liftUp();   mState=M_TO_CITY; break;           // 적재
    case M_TO_CITY:   goTo(missionDest); mState=M_CITY_WAIT; break;  // 읽은 도시로
    case M_CITY_WAIT: if(!navActive) mState=M_UNLOAD; break;         // 도착 대기
    case M_UNLOAD:    liftDown(); beep(500,150); mState=M_TO_HUB; break;  // 하차 → 반복
  }
}

//==================== 버튼 비상정지 ====================
void checkStopButton(){
  if(digitalRead(pinButton)!=0) return;
  Stop(); delay(20);
  if(digitalRead(pinButton)!=0) return;
  while(digitalRead(pinButton)==0) delay(5);
  running=!running;
  beep(running?880:300, 150);
}

//==================== setup / loop ====================
void setup(){
  Serial.begin(9600);
  pinMode(pinLTLeft,INPUT); pinMode(pinLTRight,INPUT); pinMode(pinFrontC,INPUT);
  pinMode(pinButton,INPUT_PULLUP);
  pinMode(pinBuzzer,OUTPUT);
  pinMode(pinDIR1,OUTPUT); pinMode(pinPWM1,OUTPUT);
  pinMode(pinDIR2,OUTPUT); pinMode(pinPWM2,OUTPUT);
  Stop();

  SPI.begin(); mfrc522.PCD_Init();
  liftDown();                       // 시작: 리프트 내림 후 서보 분리

  START = {START_ROW, START_COL};
  START_DIR = (START_COL==0) ? EAST : WEST;
  robotPos = START; robotDir = START_DIR;

  delay(300); beep(523,150);
  Serial.println(F("Ready - 시작 카드를 태깅하세요"));
}

void loop(){
  updateBeep();

  if(!started){                              // 시작 전: 시작 카드 대기
    byte uid[4];
    if(readUID(uid)){
      Serial.print(F("UID:"));
      for(int i=0;i<4;i++){ Serial.print(' '); if(uid[i]<0x10) Serial.print('0'); Serial.print(uid[i],HEX); }
      if(isStartCard(uid)){ started=true; running=true; beep(880,150); Serial.println(F(" => START")); }
      else                { beep(200,200);                              Serial.println(F(" => 불일치")); }
    }
    Stop(); return;
  }

  checkStopButton();                         // 시작 후: 버튼 = 정지/재개
  if(!running){ Stop(); return; }

  runMoveTo();                               // 주행
  runMission();                              // 미션 진행
}
