#include <Servo.h>
#define ServoPin 12
#define LED 13
#define sonar_Echo_Pin 2
#define sonar_Trig_Pin 3

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

float left_distance;
float right_distance;

Servo myservo;

float sonar_detection()
{
  float distance;
  digitalWrite(sonar_Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(LED, 1);
  digitalWrite(sonar_Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_Trig_Pin, LOW);
  digitalWrite(LED, 0);
  distance = pulseIn(sonar_Echo_Pin, HIGH, 8000) / 58.00;
  if (distance >= 130 || distance == 0)
  {
    distance = 130;
  }

  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  delay(5);

  return distance;
}

void motion(int speed_A, int speed_B) //电机速度控制函数。括号内分别为左右电机速度值，直接作用于L9110S电机驱动
{
  if (speed_A > 0) //范围-255～+255，正值为正转，负值为反转。
  {
    digitalWrite(A_1A, 0);
    analogWrite(A_1B, speed_A); //PWM控制速度
  }
  else
  {
    digitalWrite(A_1A, 1);
    analogWrite(A_1B, 255 + speed_A); //PWM控制速度
  }
  if (speed_B > 0)
  {
    analogWrite(B_1A, speed_B); //PWM控制速度
    digitalWrite(B_1B, 0);
  }

  else
  {
    analogWrite(B_1A, 255 + speed_B); //PWM控制速度
    digitalWrite(B_1B, 1);
  }
}

void setup()
{
  Serial.begin(9600);    //打开串口设置波特率
  pinMode(A_1A, OUTPUT); //配置电机输出IO口为输出
  pinMode(A_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);

  pinMode(sonar_Trig_Pin, OUTPUT);
  pinMode(sonar_Echo_Pin, INPUT);

  myservo.attach(ServoPin);
  myservo.write(180); //left
  delay(500);
  myservo.write(0); //right
  delay(500);
  myservo.write(73); //positive
  delay(500);
}

void loop()
{
  myservo.write(73); //positive

  if (sonar_detection() > 35)
  {
    motion(200, 200);
  }

  else
  {
    motion(0, 0);
    myservo.write(0); //right
    delay(500);
    right_distance = sonar_detection();
    myservo.write(180); //left
    delay(500);
    left_distance = sonar_detection();
    myservo.write(73); //positive
    if (left_distance > right_distance)
    {
      motion(-255, 255);
      delay(400);
    }
    else if (left_distance <= right_distance)
    {
      motion(255, -255);
      delay(400);
    }
  }
}