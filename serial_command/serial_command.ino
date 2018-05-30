char cmd[7] = "000,000";
String cmdd = " ";
int index = 0;
int x,y;
char *p;

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = Serial.read();
    if (isDigit(inChar) || inChar == ',')
    {
     cmd[index] = inChar;
     index++;
     if(index>7)index=7;
    }
    else if (inChar == ']')
    { 
       p = strtok(cmd, ",");
       if(p==NULL)
            return;
       x=atoi(p);
       p = strtok(NULL, ",");
       if(p==NULL)
            return;
       y=atoi(p);
     
      Serial.print(x);
      Serial.print("   ");
      Serial.print(y);
      Serial.println();

      for(int i=7;i>0;i--)
      {
      cmd[i] = "";
      }
      index = 0;
    }
    
  }
}


void setup()
{
  Serial.begin(9600);
}

void loop()
{

}
