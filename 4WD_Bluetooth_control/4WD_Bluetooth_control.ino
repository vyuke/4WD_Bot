char cmd[7] = "128,128";
String cmdd = " ";
int index = 0;
int x=128, y=128;
char *p;

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

void serialEvent()
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

      // Serial.print(x);
      // Serial.print("   ");
      // Serial.print(y);
      // Serial.println();

      for (int i = 7; i > 0; i--)
      {
        cmd[i] = "";
      }
      index = 0;
    }
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
{ y = map(y, 0, 255, -255, 255);
  // if (y = 128)
  // {
  //   motion(0, 0);
  // }
  // else if (y < 128)
  // {
  //   y = map(y, 0, 128, -255, 0);
  // }
  // else if (y > 128)
  // {
  //   y = map(y, 128, 255, 0, 255);
  // }
  // motion(y, y);
  // delay(10);
  
  // if (y > 0)
  // {
  //   y = map(y, 0, 128, 0, 255);
  // }
  // else if (y < 0)
  // {
  //   y = map(y, 0, -128, 0, -255);
  // }
  // else
  // {
  //   y = 0;
  // }

  Serial.print(x);
  Serial.print("   ");
  Serial.print(y);
  Serial.println();

  motion(0, 0);
  delay(1);
}
