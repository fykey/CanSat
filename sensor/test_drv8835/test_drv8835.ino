//input
//5,8
#define RIGHT_EN 6
#define RIGHT_PH 7
//6,7
#define LEFT_EN 5
#define LEFT_PH 8


void setup() 
{ 
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_PH, OUTPUT);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_PH, OUTPUT);
  
}

void loop() 
{ 
  //左右2秒間正転
  
  digitalWrite(RIGHT_PH,HIGH);        
  analogWrite(RIGHT_EN, 150);   //PWM Speed Controlz

  
  digitalWrite(LEFT_PH,HIGH);        
  analogWrite(LEFT_EN, 150);   //PWM Speed Control

  delay(2000);

  //左右2秒間静止
  digitalWrite(RIGHT_PH,LOW);        
  analogWrite(RIGHT_EN,0);   //PWM Speed Control
  
  
  digitalWrite(LEFT_PH,LOW);        
  analogWrite(LEFT_EN,0);   //PWM Speed Control

  delay(2000);

  //左右2秒間逆転
  
  digitalWrite(RIGHT_PH,LOW);        
  analogWrite(RIGHT_EN, 150);   //PWM Speed Control
  
  
  digitalWrite(LEFT_PH,LOW);        
  analogWrite(LEFT_EN, 150);   //PWM Speed Control
  delay(2000);

  

  //左右2秒間静止
  digitalWrite(RIGHT_PH,LOW);        
  analogWrite(RIGHT_EN,0);   //PWM Speed Control
  
  
  digitalWrite(LEFT_PH,LOW);        
  analogWrite(LEFT_EN,0);   //PWM Speed Control

  delay(2000);
  
}
