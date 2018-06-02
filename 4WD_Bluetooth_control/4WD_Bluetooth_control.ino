/*此示例必须与Blinker app联动，下载方式和更多相关资料请访问https://blinker.app/   */

//更多资料欢迎访问我们的官网 http://openjumper.cn/ 或联系邮箱 support@openjumper.com
//https://openjumper.taobao.com/ --OPENJUMPER官方店铺
//https://shop555818949.taobao.com/ --OPENJUMPER企业店铺


char cmd[7] = "128,128";  //定义一个数组用于抓取蓝牙摇杆的信号
int index = 0; //定义一个数用于对数组进行操控
int x = 128, y = 128;  //定义两个变量存储蓝牙摇杆的信号值
int R_motor = 0;  //定义右轮速度值
int L_motor = 0; //定义左轮速度值
char *p;  //定义一个指针用于分离字符串中的指令

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

void serialEvent()   //当串口有字符串传入后触发函数
{
  while (Serial.available())
  {
    char inChar = Serial.read();
    if (isDigit(inChar) || inChar == ',')
    {
      cmd[index] = inChar;
      index++;
      if (index > 7)
        index = 7;
    }
    else if (inChar == ']')
    {
      p = strtok(cmd, ",");
      if (p == NULL)
        return;
      x = atoi(p);
      p = strtok(NULL, ",");
      if (p == NULL)
        return;
      y = atoi(p);

      Convert(x, y);

      for (int i = 7; i > 0; i--)
      {
        cmd[i] = "";
      }
      index = 0;
    }
  }
}

void Convert(int x_val, int y_val)//将摇杆向量转化为左右驱动轮速度
{ 
  R_motor = 0;//初始化轮速
  L_motor = 0;
  if(x_val==128||y_val==128)  //摇杆处于中间位置时速度为零
  {
    R_motor = 0;
    L_motor = 0;
  }
  else
  {
    R_motor -= map(x_val, 0, 255, -255, 255);//将摇杆的向量模分配到左右两轮的轮速，有优化的可能
    R_motor += map(y_val, 0, 255, -255, 255);
    L_motor += map(x_val, 0, 255, -255, 255);
    L_motor += map(y_val, 0, 255, -255, 255);
  }

  if (R_motor > 255)
  {
    R_motor = 255;
  }
  else if (R_motor < -255)
  {
    R_motor = -255;
  }
  else if (L_motor > 255)
  {
    L_motor = 255;
  }
  else if (L_motor < -255)
  {
    L_motor = -255;
  }
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
}

void loop()
{

  Serial.print(L_motor);//串口返回轮速数据
  Serial.print("   ");
  Serial.print(R_motor);
  Serial.println();

  motion(L_motor, R_motor);//将轮速数据传递到动力函数中
  delay(1);
}
