#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include "Arduino.h"

enum SwitchType
{
	kNormallyOpen,
	kNormallyClosed
};

class LimitSwitch
{
public:
	LimitSwitch(int port, SwitchType type = kNormallyOpen);
	boolean Pressed();
private:
	int m_switchPort;
	SwitchType m_switchMode;
};

#endif