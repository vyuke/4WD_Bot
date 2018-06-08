//更多资料欢迎访问我们的官网 http://openjumper.cn/ 或联系邮箱 support@openjumper.com
//https://openjumper.taobao.com/ --OPENJUMPER官方店铺
//https://shop555818949.taobao.com/ --OPENJUMPER企业店铺

#define left_IR_switch 16  //左边的巡线传感器链接到A2(pin 16)
#define right_IR_switch 17 //右边的巡线传感器链接到A3(pin 17)

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

int IR_switch_state[2]; //存放两个巡线传感器读到的状态

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
  pinMode(left_IR_switch, INPUT); //配置2个传感器IO口为输入
  pinMode(right_IR_switch, INPUT);
  Serial.begin(9600);    //打开串口设置波特率
  pinMode(A_1A, OUTPUT); //配置电机输出IO口为输出
  pinMode(A_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
}
void loop()
{
  IR_switch_state[0] = digitalRead(left_IR_switch); //分别为从左到右2个红外寻线传感器
  IR_switch_state[1] = digitalRead(right_IR_switch);

  for (int i = 0; i < 2; i++)
  {
    Serial.print(IR_switch_state[i]); //串口输出传感器当前的状态
  }
  Serial.print("\n"); //换行

  if ((IR_switch_state[0] == 0) && IR_switch_state[1]) //左端传感器检测到障碍物
  {

    motion(255, 0); //右转
  }
  else if (IR_switch_state[0] && (IR_switch_state[1] == 0)) //右端传感器检测到障碍物
  {

    motion(0, 255); //左转
  }
  else if ((IR_switch_state[0] == 0) && (IR_switch_state[1] == 0)) //lost line
  {
    motion(-100, -255); //后退
  }
  else if (IR_switch_state[0] && IR_switch_state[1]) //on line
  {
    motion(255, 255); //全速前进
  }

  delay(10);
}
