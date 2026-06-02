/****************************************************************
 * Jellibi AGV - RFID 단독 테스트 (목차 8단계)
 *   · RC522(MFRC522) 리더로 태그 UID를 읽어 시리얼 출력
 *   · 핀(SPI): SS=D2, RST=D4, MOSI=D11, MISO=D12, SCK=D13  (Jellibi_AGV2 핀맵)
 *
 * [준비] Arduino IDE → 라이브러리 매니저 → "MFRC522" (by GithubCommunity) 설치
 *
 * [사용법]
 *   1. 업로드 → 시리얼 모니터(9600)
 *   2. 시작 시 리더 버전이 찍힘 (0x00/0xFF 이면 배선/전원 문제)
 *   3. 태그를 리더에 가까이 대면 UID(hex) 출력
 *   4. 각 도시 태그의 UID를 기록 → 베이스의 cityTable 에 입력
 *      (출력에 cityTable 복사용 형식도 같이 찍힘)
 ****************************************************************/
#include <SPI.h>
#include <MFRC522.h>

#define RFID_SS   2   // D2 (SDA/SS)
#define RFID_RST  4   // D4 (Reset)

MFRC522 mfrc522(RFID_SS, RFID_RST);

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  delay(50);

  Serial.println(F("=== RFID Test (RC522) ==="));
  Serial.print(F("Reader Version: "));
  mfrc522.PCD_DumpVersionToSerial();   // 0x91/0x92 등 정상, 0x00/0xFF 면 연결 문제
  Serial.println(F("태그를 리더에 대세요..."));
}

void loop() {
  // 새 카드가 없거나 읽기 실패하면 대기
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())  return;

  // 1) 사람이 읽는 UID
  Serial.print(F("UID:"));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(' ');
    if (mfrc522.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }

  // 2) cityTable 복사용 형식:  { {0xXX,0xXX,0xXX,0xXX}, {row,col} },
  Serial.print(F("   →  { {0x"));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) Serial.print(F(",0x"));
  }
  Serial.println(F("}, {row,col} },"));

  mfrc522.PICC_HaltA();   // 카드 통신 종료
  delay(800);             // 같은 태그 중복 출력 방지
}
