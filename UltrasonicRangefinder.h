#ifndef ULTRASONICRANGEFINDER_H
#define ULTRASONICRANGEFINDER_H
#include "Arduino.h"

class UltrasonicRangefinder {
public:
	UltrasonicRangefinder(int port);
	float GetDistanceInCM();
	float GetDistanceInInches();
private:
	int m_rangefinderPort;
};

#endif