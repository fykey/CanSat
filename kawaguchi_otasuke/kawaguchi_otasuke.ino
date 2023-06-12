#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Serial.h>
#include <Servo.h>
#include <SD.h>
#include <String.h>


#define landing_threshold 9.0
#define heght_threshold   2000

// コメントアウト

/*
 * BMXのサンプルコードが乗ってますので，配線と一緒に確認してください．
 * https://akizukidenshi.com/catalog/g/gK-13010/
 */
// BMX055 加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)


/******************************************************************************
 * PIN アサインは確認してください
 ******************************************************************************/
// Serial for GPS
#define RXD_PIN 2
#define TXD_PIN 3

//SPI for SD
#define SS      10
#define MOSI    11
#define MISO    12
#define SCK     13

//I2C for DC
#define EN_1    5
#define EN_2    6
#define PH_1    7
#define PH_2    8

#define SRV_PIN 9

// 目標位置　適宜変更すること
#define TARGET_LATITUDE     100
#define TARGET_LONGITUDE    200

SoftwareSerial GPSSerial(2,3);
File file;

float ReadAccelABS();
float ReadMagnet(float& mx, float& my, float& mz);
void ReadGPS(float& pos_x, float& pos_y, float& pos_z);
/*
 * 参照渡しを使って値を渡す．知らない人はこれを見てください
 * https://qiita.com/agate-pris/items/05948b7d33f3e88b8967
 */


void DC_Flont(int millimeters);
void DC_Richt(int degree);
void DC_Left(int degree);
void DC_Back(int millimeters);

void SD_Write(float gps_ata);
void CompassCalibration(float &cal_x, float &cal_y, float &cal_z);




/*
 * 命名規則
 * 関数名：キャメル
 * 変数名：スネークケース
 * メンバ変数　A_
 * プライベート変数 B_
 * 
 */
int calib_dir[3];
int calib_deg;
int cal_x, cal_y, cal_z;

 
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);//シリアル速度は状況に応じて変えてください
  GPSSerial.begin(115200);//シリアル速度は状況に応じて変えてください
  
  if(!SD.begin(SS)){
    Serial.println("Card Mount Failed");
    return;
  }

  
  
  /*
   * 着陸判定
   * GPSや加速度を読み取って判定
   * GPS参考サイト
   * https://www.petitmonte.com/robot/howto_gysfdmaxb.html
   * ここからスタート
   */
   while(true){
   float A = ReadAccelABS();
   ReadGPS(float pos_x, float pos_y, float pos_z);
   if A > landing_threshold && GPS < heght_threshold){
    delay(10);
    continue;
   }else { 
    break;
   }
   }
   /*
    * ちゃくりくはんていおわり　
    */

   Servo.write(180);
   DC_flont(5000);

   delay(1000);
   
   /*
    * 機体をを360度回転させて，BMXを一周させる．このときのゼロ点のずれを確認する
    */
   CompassCalibration(cal_x,cal_y, cal_z);
   ReadMagnet(int mx, int my, int mz);

   calib_dir[0] = mx - cal_x;
   calib_dir[1] = my- cal_y;
   calib_dir[2] = mz- cal_z;

   calib_deg = atan2(my/mx); // 現在姿勢に対する，基準方向
   

}
int count_calib = 0;

void loop() {
  // put your main code here, to run repeatedly:

  /*
   * 現在位置の測定
   */
   
   ReadMagnet(float mx, float my, float mz);
   float direc_now = atan2(my/mx);
   float correc_angle = calib_deg - direc_now;

   bool correc_flag = (abs(correc_angle>= 10)? true : false);     
   //誤差が前後10度以内なら，補正しない
   //三項演算子っていうifの書式です
  /*
   * DCの制御
   * 旋回および直進を実行する
   */
   
    if (correc_flag){
     if(correction_angle >= 0){
      DC_Left(int(correc_angle));
     }else{
      DC_Right(int(-correc_angle));
     }
    }

   DC_Flont(10000); // 10 m　走る

   /*
    * GPSの読み取り．SDにも格納する
    */
    
    ReadGPS(float pos_x, float pos_y, float pos_z);
    String data = String(pos_x) + "," + String(pos_y) + "," + String(pos_z) + "\n";
    file = SD.open("GPS.csv", FILE_WRITE);
    if(file){
      file.write(data)
    }
    file.close();
}




float readAccelABS(){
   float ax = digitalRead(ax_pin);
   float ay = digitalRead(ay_pin);
   float az = digitalRead(ay_pin);
   float A = sqrt(ax * ax + ay * ay + az * az);
   return A
}
