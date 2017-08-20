
#include <ax12.h>

int numOfJoints = 5;


void setup()
{
  Serial.begin(38400);

  // Set max torque
  //ax12SetRegister2(254, 14, 512);
  //delay(10);

  // Set punch
  //ax12SetRegister2(254, 48, 100);
  //delay(10);

  // Set compliance margins
  //ax12SetRegister(254, 26, 1);
  //ax12SetRegister(254, 27, 1);
  //delay(10);

  // Set compliance slopes
  ax12SetRegister(254, 28, 50);
  ax12SetRegister(254, 29, 50);
  delay(10);

  // Set goal speed
  //ax12SetRegister2(254, 32, 0);
  //delay(10);

  // Home motors
  SetPosition(254, 512);
  delay(10);
}


void loop()
{
}


void serialEvent()
{
  int id[numOfJoints];
  int pos[numOfJoints];
  while (Serial.available())
  {
    for (int i = 0; i < numOfJoints; ++i)
    {
      id[i] = Serial.parseInt();
      pos[i] = Serial.parseInt();
    }
    if (Serial.read() == '\n')
    {
      for (int i = 0; i < numOfJoints; ++i)
      {
        if ( (1 <= id[i]) && (id[i] <= numOfJoints) && (0 <= pos[i]) && (pos[i] <= 1023 ) )
          SetPosition(id[i], pos[i]);
        //Serial.print("I got id: ");
        //Serial.println(id[i]);
        //Serial.print("I got pos: ");
        //Serial.println(pos[i]);
      }
    }
  }
}


// Alternative way:
//void serialEvent()
//{
//  int id[numOfJoints];
//  int pos[numOfJoints];
//  char buffer[256];
//  int i = 0;
//  while (Serial.available())
//  {
//    char c = Serial.read();
//    if(c == '\n')
//    {
//      int j = 0;
//      int count = 0;
//      char *p = buffer;
//      char *str;
//      //Serial.println("in parser:");
//      //Serial.println(buffer);
//      while ( (str = strtok_r(p, ",", &p)) )
//      {
//        // Odd numbers are IDs, even numbers are positions
//        if (j % 2 == 0)
//        {
//          id[count] = strtol(str, NULL, 10);
//          //Serial.println(id[count]);
//          count++;
//        }
//        else
//        {
//          pos[count] = strtol(str, NULL, 10);
//          //Serial.println(pos[count]);
//        }
//        j++;
//      }
//      i = 0;
//      buffer[i] = NULL;
//    }
//    else
//    {
//      //Serial.println("adding to buffer:");
//      //Serial.println(buffer);
//      buffer[i++] = c;
//      buffer[i] = '\0';
//      delay(1);
//    }
//  }
//}

