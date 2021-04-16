//ประกาศ//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//GPS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ประกาศGPS
#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BT.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#define GP2 (2)
#define RXPin (16)
#define TXPin (17)
#define pi 3.14
#define D0 13
#define D1 10
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(2);
//float LatBoard=14.036248100052074;//ทดสอบหน้าภาค14.036248100052074, 100.72542509169298
//float LonBoard=100.72542509169298;
//float  LatPhone =14.0350593;//ทดสอบสนามกีฬา14.0350593, 100.7226041
//float  LonPhone = 100.7226041;
float LatPhone=0;
float LonPhone=0;
float LatBoard;
float LonBoard;
float LatB;
float LonB;
float LatP;
float LonP;
int DGeo;
int cm;
float DD;
int Sec;
int start=0;
int it =1;
int finish=0;
float Disture;
int f=0;
int b=0;
int t=0;
int IN;
bool INs;
int nQ;
int Lang;
int aoo;
int boo;
bool flag1 = 0;
bool flag2 = 0;
float Mlon;
float Mlat;
bool Ggets=0;
int A1;
int ANGLE = boo;
int M;
int N;
int K;
int L;
char led1 = D0;
char led2 = D1;
int text_In;
// char auth[] = "w2EG5i50amEkVRfqlt1-U6lwv6mZJJUq";
char auth[] = "yncA5E_y8AhWSxr3j6YYQ90hawAzQ14z";
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ประกาศL298N++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ประกาศL298N
#define IN1 27
#define IN2 26
#define IN3 33
#define IN4 32
void setPWM(int ch, int duty) {
  int new_duty = map(duty, 0, 100, 0, 1023);
  ledcWrite(ch, new_duty);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ประกาศHMC+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ประกาศHMC
#include <QMC5883LCompass.h>
QMC5883LCompass compass;
int Dc[37]={0,11,21,33,45,54,65,75,86,97,108,114,125,135,147,156,166,175,186,195,205,214,223,231,239,247,255,263,273,280,289,299,308,318,328,346,357};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ประกาศSONAR+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ประกาศSONAR
const int pingPin = 13; //trigged
int inPin = 23;  //echo
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//functionBYLNK+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++functionBYLNK
BLYNK_CONNECTED() {
  Blynk.syncAll();
}
BlynkTimer timer;
BLYNK_WRITE(V1){
  text_In = b;  // Text Input Widget - Strings
  Blynk.virtualWrite(V1,text_In);
  }
BLYNK_WRITE(V2) {
  if(Ggets == 0){
  GpsParam gps(param);
  Serial.print("LatPhone: ");
  LatPhone = gps.getLat();
  Serial.println(LatPhone, 7);
  Serial.print("LonPhone: ");
  LonPhone = gps.getLon();
  Serial.println(LonPhone, 7);
   if(LatPhone>0&&LonPhone>0){
      digitalWrite(GP2,HIGH);
      Serial.println("รับสัญญานได้แล้ว");
      }else{ digitalWrite(GP2,LOW);}
      // Print 2 decimal places for Alt, Speed
}else if(Ggets>=1){
  Ggets=Ggets++;
  }
if(Ggets>=5){
  Ggets=0;
  delay(5);
  }}
BLYNK_WRITE(V3)
{
   if (param.asInt() == 0)
  {
    digitalWrite(INs, LOW);
    finish = 0;
    delay(5);
    start=0;
    Ggets=0;
    delay(5);
    Serial.println(finish);
  }
  if (param.asInt() == 1)
  {
    digitalWrite(INs, HIGH);
    finish = 1;
    Ggets=1;
    delay(5);
    start=1;
    delay(5);
    Serial.println(finish);
  }
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup() {
  Serial.begin(115200);

//GPS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++SETUP_GPS
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(TinyGPSPlus::libraryVersion());
  Blynk.setDeviceName("Blynk");
  Blynk.begin(auth);
  pinMode(D0, OUTPUT); //กำหนดโหมด ว่าเป็น INPUT หรือ OUTPUT
  pinMode(D1, OUTPUT);
//L298N++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++SETUP_L298N
  pinMode(GP2,OUTPUT);
  ledcSetup(0, 500, 10);
  ledcSetup(1, 500, 10);
  ledcSetup(2, 500, 10);
  ledcSetup(3, 500, 10);
  ledcAttachPin(IN1, 0);
  ledcAttachPin(IN2, 1);
  ledcAttachPin(IN3, 2);
  ledcAttachPin(IN4, 3);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++SETUPHMC
  compass.init();
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++SETUPSONARA
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
}
void loop() {
  //GPS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++LOOP_GPS
  while(ss.available() > 0)
  if (gps.encode(ss.read()))
  displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
{
  Serial.println(F("No GPS detected: check wiring."));
  if(true);
}
  Blynk.run();
  timer.run();
  float Vlat = abs(LatPhone - LatBoard);
  float Vlon = abs(LonPhone - LonBoard);
  float S2lat = Vlat * Vlat;
  float S2lon = Vlon * Vlon;
  float Distance = S2lat + S2lon;
  float Disment = sqrt(Distance);
   Disture = 1000 * (Disment*108);
  float ylx = (Vlat/Vlon ) ; //lat=Y,lon=x
  float Geo = atan(ylx);  //in radians
   DGeo = (Geo * 180)/pi ; //transfer to deg
   if(LatP>LatB&&LonP>LonB){
    nQ=1;
    }else if(LatP>LatB&&LonP<LonB){
    nQ=2;
    }else if(LatP<LatB&&LonP<LonB){
      nQ= 3;
      }else if(LatP<LatB&&LonP>LonB){
        nQ = 4;
        }
  if( nQ==2){
    Lang=DGeo+270;
    }else if(nQ==3){
      Lang=270-DGeo;
      }else if(nQ==4){
        Lang=90+DGeo;
        }else{
          Lang=90-DGeo;}
  Serial.print(Lang);
  Serial.print(",");
  Serial.print("ระยะทาง=");
  Serial.print(Disture,6);
  Serial.println(" เมตร");
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++LOOPHMC
  compass.read();
  // Return Azimuth reading
  boo = compass.getAzimuth();

  for(int i = 0; i<=37; i++){
    if(boo<=Dc[i]){
      boo=map(boo,(Dc[i-1]),Dc[i],(i-1)*10,(i*10));
      i=37;
      if(boo==360){b=359;}
      }
    }
  Serial.print("A: ");
  Serial.print(boo);
  Serial.println();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++LOOPSONAR
  long duration;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(inPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
if(start==1){
Work_start();//**************************************************************************************************************starter
}else{
  setPWM(0, 0);
  setPWM(1, 0);
  setPWM(2, 0);
  setPWM(3, 0);
  Seria();
  Serial.println("waitpushButton");
}
}
long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++fucntionGPS
void displayInfo()
{
M=0;
  if (gps.location.isValid())
{
      LatBoard= gps.location.lat();
      LonBoard= gps.location.lng();
  }
  else
  {
  Serial.print(F("INVALID"));
  }
  Serial.println();
}
void Seria(){
  LatB = LatBoard;
  LonB = LonBoard;
  LatP = LatPhone;
  LonP = LonPhone;
  Serial.print("LatB,LonB =");
  Serial.print(LatB,7);
  Serial.print(",");
  Serial.println(LonB,7);
  Serial.print("LatP,LonP =");
  Serial.print(LatP,7);
  Serial.print(",");
  Serial.println(LonP,7);
}
void Left(){
  Serial.println("Moving Left");
  setPWM(0, 30);
  setPWM(1, 0);
  setPWM(2, 0);
  setPWM(3, 15);
  }
void Right(){
  Serial.println("Moving Right");
  setPWM(0, 0);
  setPWM(1, 30);
  setPWM(2, 15);
  setPWM(3, 0);
  }
void Turn(){
  if(ANGLE>N){
      Left();
      Serial.println("หมุนรถทางขวา");
      }else if(ANGLE<M){
        Right();
        Serial.println("หมุนรถทางซ้าย");
        }
  }
void Work_start(){
   M=Lang-14;
   N = Lang+14;
  if(finish == 1){
    if(cm >= 50){
      Serial.print("Sonar=");
      Serial.println(cm);
      delay(200);
      Serial.print("M=");
      Serial.println(M);
      Serial.print("N=");
      Serial.println(N);
      if(M<=boo&&boo<=N){
        Serial.print("องศาตรงกับเป้าหมาย");
        Serial.print("ANGLE=");
        Serial.println(boo);
         f = 1;
         t =0;
         b = 0;
        if(f==1){
        Serial.println("เดินหน้าหาเป้าหมาย");
        setPWM(0, 80);
        setPWM(1, 0);
        setPWM(2, 80);
        setPWM(3, 0);
        }
        if(Disture<=4){
          f = 0;
           b = 1;
           t=0;
          if(b==1){
          Serial.println("ถึงเป้าหมาย");
          setPWM(0, 0);
          setPWM(1, 0);
          setPWM(2, 0);
          setPWM(3, 0);
          finish = 0;
          }}
      }else{
         f = 0;
         b = 0;
          t = 1;
         if(t==1){
        Serial.println(boo);
              if(boo>M){
                  Left();
                  Serial.println("หมุนรถทางซ้าย");
                  }else if(boo<N){
                    Right();
                    Serial.println("หมุนรถทางขวา");
                          }
                  }
            }
          }else{
            it = 0;
            while(it==0){
              Serial.println("หยุดรถ");
              setPWM(0, 0);
              setPWM(1, 0);
              setPWM(2, 0);
              setPWM(3, 0);
              delay(1000);
             Serial.println("หมุนรถ");
              setPWM(0, 0);
              setPWM(1, 35);
              setPWM(2, 35);
              setPWM(3, 0);
              delay(1800);
             Serial.println("เดินหน้า");
              setPWM(0, 50);
              setPWM(1, 0);
              setPWM(2, 50);
              setPWM(3, 0);
              delay(3500);
              it=1;
            }
          }
  }else if(finish == 0){
  Serial.println(finish);
  Serial.println("done");
  f=0;
  b=0;
  t=0;
  Ggets=0;
  Blynk.virtualWrite(V3, 0);
  start=0;
  }
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++end
