/****************************************************************
 * Jellibi AGV - 창고↔도시 물류 왕복 (단순 미션)
 *   스타트 RFID → 창고 이동 → 물류 적재 → 도시 이동 → 물류 하차 → 창고 반복
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

//==================== ★ 여기만 수정 ====================
int  WAREHOUSE_NO = 3;     // 창고 번호 (1~8)
int  CITY_NO      = 3;     // 도시 번호 (1~8): 1서울 2인천 3세종 4대전 5대구 6광주 7부산 8제주
int  START_ROW    = 4;     // 출발 위치 행 (A=1 … H=8)
int  START_COL    = 0;     // 출발 위치 열: 0=좌측출발, 9=우측출발
byte START_CARD[4] = {0x74,0xDC,0x58,0x74};  // 시작 카드 UID (RfidTest로 확인)
int  LIFT_UP_ANGLE   = 150; // 적재(올림) 각도
int  LIFT_DOWN_ANGLE = 40;  // 하차(내림) 각도
int  actionDelayMs   = 1000;// 적재/하차 동작 시간(ms)

//==================== 튜닝 상수 ====================
#define LEFT_WHITE   415    // 센서 정규화(실측)
#define LEFT_BLACK   992
#define RIGHT_WHITE  378
#define RIGHT_BLACK  959
#define LINE_COUNT_THRESHOLD 700   // 교차점 판정(둘 다 이상)
#define LINE_LEAVE_THRESHOLD 450   // 벗어남 판정(둘 다 미만)

#define FORWARD  0
#define BACKWARD 1

int   basePower = 110, minPower = 75, maxPower = 145;
float Kp = 0.015, Power1RatioF = 0.99, Power2RatioF = 1.00;
int   turnPower = 100;
int   forwardIntoCrossTime = 50;
int   forwardAfterTurnTime = 0;
int   forwardAfterHalfTurnTime = 60;
int   leftTurnTime = 420, rightTurnTime = 420, halfTurnTime = 860;
int   arriveCrossForwardTime = 60;

//==================== 좌표계 ====================
//  행: 0=창고줄, 1~8=격자 A~H, 9=도시줄 / 열: 0=좌출발, 1~8=격자, 9=우출발
struct Point { int row, col; };
enum Direction { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
#define GRID_RMIN 1
#define GRID_RMAX 8
#define GRID_CMIN 1
#define GRID_CMAX 8

Point     HUB, DEST, START;
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

//==================== RFID / 서보 ====================
MFRC522 mfrc522(pinRfidSS, pinRfidRST);
Servo   liftServo;

//==================== 상태 변수 ====================
int  lineCounter=0; bool lineLock=false;
unsigned long lastCountTime=0;
int  lineCountIgnore=320, leaveNeed=2, leaveCount=0, crossNeed=2;

int  targetCount=0; bool traceDone=false;     // DoLineTrace
int  moveState=0; Point moveTarget;            // MoveTo
Point wp[4]; int wpLen=0, wpIdx=0; bool navActive=false;  // 경로
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
void ignoreAfterTurn(){ resetCounter(); lineLock=true; }   // 회전점 교차로 1개 스킵
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

Point gateway(Point p){                 // 스텁 → 격자 진입점
  Point g=p;
  if      (p.row==0) g.row=GRID_RMIN;    // 창고(위)  → 아래
  else if (p.row==9) g.row=GRID_RMAX;    // 도시(아래)→ 위
  else if (p.col==0) g.col=GRID_CMIN;    // 좌출발 → 오른쪽
  else if (p.col==9) g.col=GRID_CMAX;    // 우출발 → 왼쪽
  return g;
}
int planRoute(Point from, Point to){     // [스텁진입]→격자이동→[스텁탈출]
  int n=0; wp[n++]=from;
  Point gA=gateway(from), gB=gateway(to);
  if(!samePoint(gA,from)) wp[n++]=gA;
  if(!samePoint(gB,gA))   wp[n++]=gB;
  if(!samePoint(to,gB))   wp[n++]=to;
  return n;
}
void goTo(Point dest){
  wpLen=planRoute(robotPos,dest); wpIdx=1; navActive=true;
  if(wpIdx<wpLen) startMove(wp[wpIdx]); else navActive=false;
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
    if(wpIdx<wpLen) startMove(wp[wpIdx]); else navActive=false;  // 목적지 도착
  }
}

//==================== RFID 시작 카드 ====================
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

//==================== 리프트 ====================
void liftUp(){   liftServo.attach(pinLiftServo); liftServo.write(LIFT_UP_ANGLE);   delay(actionDelayMs); }                       // 적재: 부착·올림(운반 중 유지)
void liftDown(){ liftServo.attach(pinLiftServo); liftServo.write(LIFT_DOWN_ANGLE); delay(actionDelayMs); liftServo.detach(); }  // 하차: 내린 뒤 서보 분리(전류↓→리셋 방지)

//==================== 미션: 창고→적재→도시→하차→반복 ====================
enum { M_TO_HUB, M_HUB_WAIT, M_LOAD, M_TO_CITY, M_CITY_WAIT, M_UNLOAD };
int mState = M_TO_HUB;

void runMission(){
  switch(mState){
    case M_TO_HUB:    goTo(HUB);  mState=M_HUB_WAIT;  break;   // 창고로
    case M_HUB_WAIT:  if(!navActive) mState=M_LOAD;   break;   // 도착 대기
    case M_LOAD:      liftUp();   beep(1200,100); mState=M_TO_CITY; break;  // 적재(서보 유지)
    case M_TO_CITY:   goTo(DEST); mState=M_CITY_WAIT; break;   // 도시로
    case M_CITY_WAIT: if(!navActive) mState=M_UNLOAD; break;   // 도착 대기
    case M_UNLOAD:    liftDown(); beep(500,150);  mState=M_TO_HUB;  break;  // 하차(서보 분리) → 반복
  }
}

//==================== 버튼 비상정지 ====================
void checkStopButton(){
  if(digitalRead(pinButton)!=0) return;          // 안 눌림(풀업)
  Stop(); delay(20);
  if(digitalRead(pinButton)!=0) return;
  while(digitalRead(pinButton)==0) delay(5);     // 손 뗄 때까지
  running=!running;
  beep(running?880:300, 150);
}

//==================== setup / loop ====================
void setup(){
  Serial.begin(9600);
  pinMode(pinLTLeft,INPUT); pinMode(pinLTRight,INPUT);
  pinMode(pinButton,INPUT_PULLUP);
  pinMode(pinBuzzer,OUTPUT);
  pinMode(pinDIR1,OUTPUT); pinMode(pinPWM1,OUTPUT);
  pinMode(pinDIR2,OUTPUT); pinMode(pinPWM2,OUTPUT);
  Stop();

  SPI.begin(); mfrc522.PCD_Init();
  liftDown();                       // 시작: 리프트 내림 후 서보 분리(idle 전류 0)

  HUB   = {0, WAREHOUSE_NO};               // 창고 N (행0)
  DEST  = {9, CITY_NO};                     // 도시 N (행9)
  START = {START_ROW, START_COL};
  START_DIR = (START_COL==0) ? EAST : WEST; // 좌출발=동, 우출발=서
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
