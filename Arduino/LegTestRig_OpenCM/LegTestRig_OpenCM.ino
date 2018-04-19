#include <string.h>

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <- OpenCM 485 EXP

int numOfJoints = 24;
char cmdBuffer[64];
int bufferCount = 0;
Dynamixel Dxl(DXL_BUS_SERIAL3);


void setup()
{
  // Initialize the Dynamixel bus
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps
  Dxl.begin(3);

  // Set joint mode
  for (int i = 0; i < numOfJoints; ++i)
    Dxl.jointMode(i);

  // Set compliance
  Dxl.complianceMargin(254, 1, 1);
  Dxl.complianceSlope(254, 50, 50);

  // Home motors
  Dxl.setPosition(254, 512, 100);

  // Setup serial
  SerialUSB.begin();
  SerialUSB.attachInterrupt(usbInterrupt);
}


void loop()
{
}


void usbInterrupt(byte* buffer, byte nCount){
  // USB max packet data is 64 bytes, so nCount cannot exceed 64 bytes
  for(unsigned int i=0; i < nCount; ++i)
  {
    char c = (char)buffer[i];
    cmdBuffer[bufferCount++] = c;
    if (c == '\n')
    {
      useBuffer(cmdBuffer);
      buffer[0] = NULL;
      bufferCount = 0;
    }
  }
}


void useBuffer(char *p)
{
  char * pEnd;
  int id;
  int pos;
  int speed;

  // String format: ID POS SPEED\n
  id = strtol(p, &pEnd, 10);
  pos = strtol(pEnd, &pEnd, 10);
  speed = strtol(pEnd, NULL, 10);

  /*
  SerialUSB.print("I got id: ");
  SerialUSB.println(id);
  SerialUSB.print("I got pos: ");
  SerialUSB.println(pos);
  SerialUSB.print("I got speed: ");
  SerialUSB.println(speed);
  */

  if ( (1 <= id) && (id <= numOfJoints) &&
       (0 <= pos) && (pos <= 1023) &&
       (0 <= speed) && (speed <= 1023) )
  {
     Dxl.setPosition(id, pos, speed);
     SerialUSB.println("Sent to AX-12!");
  }
}

