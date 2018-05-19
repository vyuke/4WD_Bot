/*4WD laser radar car with the RPLIDAR Arduino Example
 * 
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3 
 */

/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>
#include <SoftwareSerial.h>

RPLidar lidar;
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor. \
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal
// #define ARDUINO_RX1 14
// #define ARDUINO_TX1 15
// SoftwareSerial mySerial1(ARDUINO_RX1, ARDUINO_TX1);

#define A_1A 4 //A组电机正反转控制
#define A_1B 5 //PWM
#define B_1A 6 //PMW//B组电机正反转控制
#define B_1B 7 //

// void serialEvent()
// {
//   while (mySerial1.available()) //检查当前串口缓冲区是否有内容
//   {
//     Serial.println(mySerial1.read());
//   }
// }

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

  // Serial.begin(115200);
  // mySerial1.begin(9600);

  lidar.begin(Serial);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  pinMode(A_1A, OUTPUT); //配置电机输出IO口为输出
  pinMode(A_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
}

void loop()
{
  if (IS_OK(lidar.waitPoint()))
  {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle = lidar.getCurrentPoint().angle;       //anglue value in degree
    bool startBit = lidar.getCurrentPoint().startBit;  //whether this point is belong to a new scan
    byte quality = lidar.getCurrentPoint().quality;    //quality of the current measurement

    //perform data processing here...
    if ((angle > 0 && angle < 30) || (angle > 330 && angle < 360))
    {
      if (distance >= 300)
      {
        motion(255, 255);
        delay(10);
      }
      // else if (distance < 500)
      // {
      //   motion(-255, 255);
      //   delay(400);
      // }
    }
    else
    {
      if (distance < 500)
      {
        motion(0, 0);
      }
    }
  }
  else
  {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    motion(0, 0);
    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100)))
    {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}
