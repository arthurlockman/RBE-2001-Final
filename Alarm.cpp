#include "Alarm.h"

Alarm::Alarm(int pin):
	m_pin(pin)
{

}

void Alarm::On()
{
	tone(m_pin, NOTE_FS4);
}

void Alarm::Off()
{
	noTone(m_pin);
}
