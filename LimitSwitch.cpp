#include "LimitSwitch.h"

/**
 * @brief The constructor for the limit switch object.
 * @details The limit switch provides a convenient
 * access method for interfacing with limit switches 
 * and bump sensors on the robot. It provides a method
 * for reading either normally open or normally closed
 * switches.
 * 
 * @param port The port that the switch is connected to.
 * @param type What type of switch the switch is (either
 * normally open or normally closed).
 */
LimitSwitch::LimitSwitch(int port, SwitchType type):
	m_switchPort(port),
	m_switchMode(type)
{
	pinMode(m_switchPort, INPUT_PULLUP);
}

/**
 * @brief Gets whether or not the limit switch is pressed.
 * @details This method determines whether or not the 
 * switch is pressed. It takes into account what type of 
 * switch the switch is and reads it accordingly. 
 * 
 * @param  None.
 * @return A bool, whether or not it is pressed. 
 */
boolean LimitSwitch::Pressed()
{
	switch(m_switchMode)
	{
		case kNormallyOpen:
			if (digitalRead(m_switchPort) == LOW)
			{
				return true;
			}
			else
			{
				return false;
			}
			break;
		case kNormallyClosed:
			if (digitalRead(m_switchPort) == HIGH)
			{
				return true;
			}
			else
			{
				return false;
			}
			break;
	}
}
