#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(52, 53); //블루투스 모듈 연결
int recNum = -1;
Servo servo1, servo2;        //서보 모터 객체 생성
int value1 = 0;
int value2 = 0;      //서보 모터 각도

//led 연결
int led1[] = {14, 15, 16, 17, 18, 19};
int led2[] = {2, 3, 4, 5, 6, 7};
int led3[] = {8, 9, 10, 11, 12, 13};




unsigned long previousMillis = 0;   //서보모터 시간 제어
const long interval = 1000;        //서보모터 동작 주기=1sec

void setup() {
  
  Serial.begin(9600);         //시리얼 속도 설정
  BTSerial.begin(9600);       //블루투스 속도 설정  
  
  servo1.attach(30);            //서보모터를 30,31핀으로 설정
  servo2.attach(31);
  
  for (int i = 0; i < 6; i++) {
    pinMode(led1[i], OUTPUT);
    pinMode(led2[i], OUTPUT);
    pinMode(led3[i], OUTPUT);
  }
} //led 설정

void Ent(int ledPin) {
  servo1.write(0);                 // 서보 모터 열기
  digitalWrite(ledPin, HIGH);     // LED 켜기
  delay(5000);                    // 5초 대기
  servo1.write(90);                // 서보 모터 닫기
}

void EX(int LedPin){
  servo2.write(0);              // 서보 모터 열기
  digitalWrite(LedPin, LOW);    // LED 끄기
  delay(5000);                  // 5초 대기
  servo2.write(90);             // 서보 모터 닫기
}

void loop() {
  if (BTSerial.available() > 0){
    
    String rec_Data = BTSerial.readStringUntil('\n');
    rec_Data.trim();
    

    int NumIndex = rec_Data.indexOf(',');

    if (NumIndex != -1) {

      String rec_Command = rec_Data.substring(0, NumIndex);
      int recNum = rec_Data.substring(NumIndex + 1).toInt();
    }

   
    if (rec_Command == "ENTER" && recNum != -1) {       // "ENTER" 문자와 숫자를 받으면 입차 시스템 작동
      if (recNum >= 0 && recNum<6){
      
        Ent(led1[recNum]);

      }
      else if (recNum >= 6 && recNum < 12) {
        int a = recNum-6;
        Ent(led2[a]);
      }
      else if (recNum >= 12 && recNum < 18){
        int a = recNum-12;
        Ent(led3[a]);
      }
    }
  
    else if(rec_Command == "EXIT" && recNum != -1){     // "EXIT" 문자와 숫자를 받으면 출차 시스템 작동
      if (recNum >= 0 && recNum<6){
      
        EX(led1[recNum]);

      }
      else if (recNum >= 6 && recNum < 12) {
        int a = recNum-6;
        EX(led2[a]);
      }
      else if (recNum >= 12 && recNum < 18){
        int a = recNum-12;
        EX(led3[a]);
      }
    }
  }
}
