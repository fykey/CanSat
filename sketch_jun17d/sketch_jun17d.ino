#include <Wire.h>
#include "BMX_READ.h"
#include "GPS.h"
#include <SD.h>
#include <SPI.h>
#include <String.h>
#include <Servo.h>
#include <EEPROM.h>

Servo myservo;          // Servoオブジェクトの宣言
const int SV_PIN = 9;   // サーボモーターをデジタルピン9に


//I2C for DC
#define LEFT_EN    5
#define RIGHT_EN   6
#define RIGHT_PH   7
#define LEFT_PH    8


#define heght_threshold 2000  //高さの絶対値
#define land_heght 20         //高さの絶対値//

#define HEIGHT_THRESHOLD 8000
#define LANDING_THRESHOLD 1000
#define MAX_STRING_SIZE  256

int add_now = 0;


void setup() {
  Serial.begin(9600);
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  add_now = save_EEPROM(String("飛べ川口\n"));



  //BMX055 初期化
  BMX055_Init();

  delay(300);

  tyakuriku_GPS(HEIGHT_THRESHOLD, LANDING_THRESHOLD);
  GPS_to_EEPROM();

  add_now = save_EEPROM(String("着陸したンゴ\n"));


  //サーボ
  servo_setup();  //servo_return()が逆
  servo_move();

  //キャリブ　値渡しの実装要
  // put your setup code here, to run once:
  //  motor_setup()；
  //  DC_Manipulator("LEFT", 1000, 45, 150);//
  float calib_x;
  float calib_y;
  float calib_deg;
  add_now = save_EEPROM(String("回れ川口\n"));
  calibration(calib_x, calib_y, calib_deg);  //値渡し実装必要
  add_now = save_EEPROM(String("位置決まったンゴ\n"));
  add_now = save_EEPROM(String("進め川口\n"));
  DC_Manipulator("FLONT", 5000, -1, 150);
}

void loop() {  //キャリブ、走行決定、走行をここに入れるべきか　終了判定も忘れずに(距離が1 m以内とか)
  
  
}


void tyakuriku_GPS(int height_threshlod, int landing_threshold) {
  int count = 0;

  while (true) {
    float z = get_alt();
    GPS_to_EEPROM();


    if (z > HEIGHT_THRESHOLD) {
      count++;
    }
    if (count > 10) {
      //上昇確認
      for (int i = 0; i < 600; i++) {
        GPS_to_EEPROM();
        // 待機10分
        delay(1000);
      }

      while (true) {
        GPS_to_EEPROM();
        z = get_alt();
        delay(1000);
        if (z < LANDING_THRESHOLD) {
          for(int k; k < 100;k ++){
          delay(1000);
          GPS_to_EEPROM();
          }
          //着陸
          break;
        }
      }
      break;
    }
    delay(5000);
    continue;
  }
}






void DC_Manipulator(char mov, int millimeters, int degree,  int speed){
  float coeff_m = 100;// 指定距離を進むための係数．実験的に決めること
  float coeff_d = 100;// 指定角度を進むための係数．実験的に決めること
  int tim = 0;

  if (mov == "FLONT"){
      Serial.println(mov);
      delay(10);
      digitalWrite(RIGHT_PH,HIGH);  
      delay(10);      
      digitalWrite(LEFT_PH, HIGH);
      delay(10);
      tim = int(millimeters / speed * coeff_m);
      analogWrite(RIGHT_EN, speed);   //PWM Speed Control
      analogWrite(LEFT_EN, speed);   //PWM Speed Control
  
      delay(tim);


  }else if (mov == "LEFT"){
       Serial.println(mov);
      delay(10);
      digitalWrite(RIGHT_PH,HIGH); 
      delay(10);       
      digitalWrite(LEFT_PH, LOW);
      delay(10);
      analogWrite(RIGHT_EN, speed);   //PWM Speed Control
      
      tim = int(degree / speed * coeff_d);   
  }else{
      Serial.println("COMMAND NOT FOUND");    
      return;
  }
  

  analogWrite(RIGHT_EN,0);   //PWM Speed Control
  analogWrite(LEFT_EN,0);   //PWM Speed Control
  digitalWrite(RIGHT_PH,LOW);   
  digitalWrite(LEFT_PH,LOW);        
  
}

//void DC_Manipulator(String mov, long millimeters, int degree,  int speed);
void motor_setup() {
  // put your setup code here, to run once:
  //DC Motorのピン設定
  Serial.begin(9600);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_PH, OUTPUT);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_PH, OUTPUT);
}

void servo_setup(){

  myservo.attach(SV_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  
}



void servo_move(){
  myservo.write(70);    // サーボモーターを70度の位置まで動かす
  delay(1000);

  myservo.write(0);   // サーボモーターを0度の位置まで動かす
  delay(1000);
  exit(0);
}

void servo_return(){
  myservo.write(0);    // サーボモーターを0度の位置まで動かす
  delay(1000);

  myservo.write(70);   // サーボモーターを70度の位置まで動かす
  delay(1000);
  exit(0);
}


void calibration(float& calib_x, float& calib_y, float& calib_deg){
  calib_x = 0;
  calib_y = 0;
  calib_deg = 0;

  float magx[100], magy[100];


  for (int i = 0; i < 60; i++) {
    magx[i] = get_magx();
    magy[i] = get_magy();


    DC_Manipulator("LEFT", -1, 10, 150);
    delay(100);

    float max_x = 0;
    float max_y = 0;
    float min_x = 0;
    float min_y = 0;

    bool flag[4];
    for (i = 1; i < 99; i++) {

      if (magx[i - 1] <= magx[i] && magx[i] >= magx[i + 1]) {
        //極大値です！！！
        max_x = magx[i];
        flag[0] = true;
      }

      if (magy[i - 1] >= magy[i] && magy[i] >= magy[i + 1]) {
        //極大値です！！！
        max_y = magy[i];
        flag[1] = true;
      }

      if (magx[i - 1] >= magx[i] && magx[i] <= magx[i + 1]) {
        //極小値です！！！
        max_x = magx[i];
        flag[2] = true;
      }

      if (magy[i - 1] >= magy[i] && magy[i] <= magy[i + 1]) {
        //極小値です！！！
        max_y = magy[i];
        flag[3] = true;
      }

      if (flag[0] && flag[1] && flag[2] && flag[3]) {
        calib_x = max_x + min_x;
        calib_y = max_y + min_y;
        float mx = get_magx();
        float my = get_magy();

        calib_deg = atan2(my - calib_y, mx - calib_x);  // 現在姿勢に対する，基準方向
        break;
      }
    }
  }
}

void GPS_to_EEPROM(){
  String data = String(get_all());
  add_now = save_EEPROM(data);


}


int save_EEPROM(String data){
  int dataSize = data.length() + 1;
  for (int i = 0; i < dataSize; i++) {
    EEPROM.write(add_now + i, data[i]);
  }
  return add_now + MAX_STRING_SIZE;
}

String read_EEPROM(int address) {
  char dataChar[MAX_STRING_SIZE];
  for (int i = 0; i < MAX_STRING_SIZE; i++){
    dataChar[i] = EEPROM.read(address + i);
  }
  return String(dataChar);
}