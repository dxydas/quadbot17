
#include <ax12.h>

int numOfJoints = 5;

void setup()
{
  Serial.begin(38400);
}

void loop()
{
}

void serialEvent() {
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
        if ( (id[i] > 0) && (0 <= pos[i]) && (pos[i] <= 1023 ) )
          SetPosition(id[i], pos[i]);
        //Serial.print("I got id: ");
        //Serial.println(id[i]);
        //Serial.print("I got pos: ");
        //Serial.println(pos[i]);
      }
    }
  }
}

