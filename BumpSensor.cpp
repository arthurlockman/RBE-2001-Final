#include "BumpSensor.h"

BumpSensor::BumpSensor(int xPin, int yPin, int zPin, int threshold):
	m_xpin(xPin),
	m_ypin(yPin),
	m_zpin(zPin),
	m_threshold(threshold)
{
	_xlast = analogRead(m_xpin);
	_ylast = analogRead(m_ypin);
	_zlast = analogRead(m_zpin);
}

int BumpSensor::Bumped()
{
	int x = analogRead(m_xpin);
	int y = analogRead(m_ypin);
	int z = analogRead(m_zpin);
	int dx = x - _xlast;
	int dy = y - _ylast;
	int dz = z - _zlast;
	int magnitude = sqrt(sq(dx) + sq(dy) + sq(dz));

	if (magnitude > m_threshold) { return 1; }
	else  { return 0; }
}

int BumpSensor::GetX()
{
	return analogRead(m_xpin);
}

int BumpSensor::GetY()
{
	return analogRead(m_ypin);
}

int BumpSensor::GetZ()
{
	return analogRead(m_zpin);
}

void BumpSensor::SetThreshold(int threshold)
{
	m_threshold = threshold;
}
