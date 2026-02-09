#ifndef GPS_H
#define GPS_H
#include <Arduino.h>
#include "UM982Parser.h"

#define RadioRTK Serial7
#define RadioBaudRate 115200
char RTKrxbuffer[512]; // Extra serial rx buffer
byte NtripData[512];

char nmeaBuffer[200];
int count = 0;
bool stringComplete = false;
elapsedMillis lastNtrip = 0;

int test = 0;

//**************************************************************

void RTK_Setup()
{

  RadioRTK.begin(RadioBaudRate);
  RadioRTK.addMemoryForRead(RTKrxbuffer, 512);
}

//**************************************************************

void Forward_Ntrip()
{
  // Check for UDP Packet (Ntrip 2233)
  int NtripSize = NtripUdp.parsePacket();
  if (NtripSize)
  {
    // if (lastNtrip > 150) // debounce to prevent flooding
    // {
      NtripUdp.read(NtripData, NtripSize);
      SerialGPS.write(NtripData, NtripSize);
      lastNtrip = 0;
    //}
  }

  // Check for Radio RTK
  if (RadioRTK.available())
  {
    SerialGPS.write(RadioRTK.read());
  }
}

#endif