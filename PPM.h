
#ifndef PPM_h
#define PPM_h

#include "Arduino.h"

class PPM
{
  public:
    PPM(int pin);
    int getChannel(int ch);
    int getRawChannel(int ch);
    void setLow(int ch, int value);
    void setHigh(int ch, int value);
  private:

};
void updateVex();
void processVexInput();


#endif

