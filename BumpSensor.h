#ifndef BUMPSENSOR_H
#define BUMPSENSOR_H

#include "Arduino.h"

class BumpSensor
{
public:
	BumpSensor(int xPin, int yPin, int zPin, int threshold);
	int Bumped(); //return 1 if bumped, else 0
	int GetX();
	int GetY();
	int GetZ();
	void SetThreshold(int threshold);
private:
	int m_xpin;
	int m_ypin;
	int m_zpin;
	int m_threshold;
	int _xlast;
	int _ylast;
	int _zlast;
};

#endif
