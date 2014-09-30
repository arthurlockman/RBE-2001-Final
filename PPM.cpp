
#include "Arduino.h"

#include "PPM.h"


unsigned long vexInput [6] = {0,0,0,0,0,0};

int workingArray [6] = {0,0,0,0,0,0};

int mapLow [6] = {1000,1000,1000,1000,1000,1000};

int mapHigh [6] = {2000,2000,2000,2000,2000,2000};

int index=0;

unsigned long oldMicros=0;

boolean flag=false;

PPM::PPM(int pin)
{
  pinMode(pin, INPUT_PULLUP);

 
 if(pin == 2)
{

    attachInterrupt(0, updateVex, RISING);

  }

  if(pin == 3)
{

    attachInterrupt(1, updateVex, RISING);

  }
  
if(pin == 21)
{

    attachInterrupt(2, updateVex, RISING);

  }

  if(pin == 19){
    attachInterrupt(4, updateVex, RISING);

  }
  if(pin == 20){
    attachInterrupt(3, updateVex, RISING);

  }
  if(pin == 18){
    attachInterrupt(5, updateVex, RISING);

  }
}

void updateVex()
{

  unsigned long time = micros()-oldMicros;
  oldMicros=micros();

  if(time > 6000) //if low period is >6milliseconds, beginning of new series of input from controller
  {  
    index=0;
  }
  else
  {
     vexInput[index]=time; 
     index++;
     if(index == 6)
     {  
	flag=true;
     }
  }
}

int PPM::getChannel(int ch)
{
  if(flag)
  {
     processVexInput();
     flag=false; 
  }
  return workingArray[ch-1];
}

int PPM::getRawChannel(int ch)
{
  return vexInput[ch-1];
}

void processVexInput()
{
  for(int i=0; i<6; i++)
    {
       unsigned long x = vexInput[i];
       int y=map(x, mapLow[i], mapHigh[i], 0, 180);
       workingArray[i]=y;
    }   
}

void PPM::setLow(int ch, int value)
{
  mapLow[ch-1] = value;
}

void PPM::setHigh(int ch, int value)
{
  mapHigh[ch-1] = value;
}