//I2C for DC
#define LEFT_EN    5
#define RIGHT_EN   6
#define RIGHT_PH   7
#define LEFT_PH    8

void DC_Manipulator(String mov, long millimeters, int degree,  int speed);
void setup() {
  // put your setup code here, to run once:
  //DC Motorのピン設定
  Serial.begin(9600);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_PH, OUTPUT);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_PH, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  DC_Manipulator("FLONT", 1000, -1, 150);
  delay(100);

  DC_Manipulator("BACK", 1000, -1, 150);
  delay(100);

  DC_Manipulator("RIGHT", -1, 90, 150);
  delay(100);

  DC_Manipulator("LEFT", 1000, 90, 150);
  delay(100);

}


void DC_Manipulator(String mov, long millimeters, int degree,  int speed){
  float coeff_m = 10;// 指定距離を進むための係数．実験的に決めること
  float coeff_d = 10;// 指定角度を進むための係数．実験的に決めること
  int tim = 0;

  if (mov == "FLONT"){
      Serial.println(mov);
      digitalWrite(RIGHT_PH,HIGH);        
      digitalWrite(LEFT_PH, HIGH);
      tim = int(millimeters / speed * coeff_m);
  }else if (mov == "BACK"){
          Serial.println(mov);
      digitalWrite(RIGHT_PH,LOW);        
      digitalWrite(LEFT_PH, LOW);
      tim = int(millimeters / speed * coeff_m);
  }else if (mov == "RIGHT"){
          Serial.println(mov);
      digitalWrite(RIGHT_PH,LOW);        
      digitalWrite(LEFT_PH, HIGH);
      tim = int(degree / speed * coeff_d); 
  }else if (mov == "LEFT"){
          Serial.println(mov);
      digitalWrite(RIGHT_PH,HIGH);        
      digitalWrite(LEFT_PH, LOW);
      tim = int(degree / speed * coeff_d);   
  }else{
      Serial.println("COMMAND NOT FOUND");    
      return;
  }
  analogWrite(RIGHT_EN, speed);   //PWM Speed Control
  analogWrite(LEFT_EN, speed);   //PWM Speed Control
  
  //tim = 2000;
  delay(tim);

  analogWrite(RIGHT_EN,0);   //PWM Speed Control
  analogWrite(LEFT_EN,0);   //PWM Speed Control
  digitalWrite(RIGHT_PH,LOW);   
  digitalWrite(LEFT_PH,LOW);        
  
}
