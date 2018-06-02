//更多资料欢迎访问我们的官网 http://openjumper.cn/ 或联系邮箱 support@openjumper.com
//https://openjumper.taobao.com/ --OPENJUMPER官方店铺
//https://shop555818949.taobao.com/ --OPENJUMPER企业店铺

#include <Servo.h>       //调用舵机库文件
#define ServoPin 12      //定义舵机连接到12号引脚
#define LED 13           //定义LED为13号引脚
#define sonar_Echo_Pin 8 //定义超声波传感器Echo的引脚为2号口
#define sonar_Trig_Pin 9 //定义超声波传感器Trig的引脚为3号口

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

float left_distance;  //定义一个变量存储左边的距离
float right_distance; //定义一个变量存储右边的距离

Servo myservo; //声明一个对象名为myservo

float sonar_detection() //超声波测距函数
{
  float distance;                    //定义一个变量存储测得的距离
  digitalWrite(sonar_Trig_Pin, LOW); //产生一个10us的脉冲去触发Trig_Pin
  delayMicroseconds(2);
  digitalWrite(LED, 1);
  digitalWrite(sonar_Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_Trig_Pin, LOW);
  digitalWrite(LED, 0);
  distance = pulseIn(sonar_Echo_Pin, HIGH, 8000) / 58.00; //检测脉冲宽度，并计算距离值
  if (distance >= 130 || distance == 0)                   //判断如果距离大于等于130cm或者等于0，则让距离等于130
  {
    distance = 130;
  }

  Serial.print(distance); //串口输出距离值
  Serial.print("cm");     //串口输出距离值单位
  Serial.println();       //换行
  delay(5);               //延时5秒

  return distance; //返回距离值
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

  pinMode(sonar_Trig_Pin, OUTPUT); //配置Trig端口为输出模式
  pinMode(sonar_Echo_Pin, INPUT);  //配置Echo端口为输入模式

  myservo.attach(ServoPin); //初始化舵机对象连接的引脚
  myservo.write(180);       //left
  delay(500);
  myservo.write(0); //right
  delay(500);
  myservo.write(73); //positive
  delay(500);
}

void loop()
{
  myservo.write(73); //positive

  if (sonar_detection() > 35) //如果距离大于35cm则一直前进
  {
    motion(200, 200);
  }

  else
  {
    motion(0, 0);     //否则停止
    myservo.write(0); //right
    delay(500);
    right_distance = sonar_detection(); //探测右边的距离值
    myservo.write(180);                 //left
    delay(500);
    left_distance = sonar_detection();  //探测左边的距离值
    myservo.write(73);                  //positive
    if (left_distance > right_distance) //如果左边的距离大于右边的距离
    {
      motion(-255, 255); //左转400毫秒
      delay(400);
    }
    else if (left_distance <= right_distance) //如果左边的距离小于等于右边的距离
    {
      motion(255, -255); //右转400毫秒
      delay(400);
    }
  }
}