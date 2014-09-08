#include "UltrasonicRangefinder.h"
#include "Arduino.h"

/**
 * @brief An interface for the ultrasonic rangefinder.
 * @details This class provides an interface for the
 * Sharp model 2Y0A21 ultrasonic rangefinder. It has
 * the ability to measure in both inches and CM.
 * 
 * @param port The analog port that the sensor is 
 * attached to.
 */
UltrasonicRangefinder::UltrasonicRangefinder(int port) : 
	m_rangefinderPort(port)
{

}

/**
 * @brief Get distance in CM.
 * @details Get the distance to the nearest surface
 * in centimeters. Accurate from 10-80 cm.
 * 
 * @return The distance as a float, in CM.
 */
float UltrasonicRangefinder::GetDistanceInCM()
{
	int sensorValue = analogRead(m_rangefinderPort);
	return (4800.0 / (sensorValue - 20.0));
}

/**
 * @brief Get distance in inches.
 * @details Get the distance to the nearest surface
 * in inches. Accurate from ~4-31.5 inches.
 * 
 * @return The distance as a float, in inches.
 */
float UltrasonicRangefinder::GetDistanceInInches()
{
	return this->GetDistanceInCM() * 0.39370;
}
