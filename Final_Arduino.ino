#include "Robotmap.h"
#include <QTRSensors.h>

//Alarm radiationAlarm(kRadiationAlarm);

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {kLineSensor0, kLineSensor1, kLineSensor2, 
	kLineSensor3, kLineSensor4, kLineSensor5, kLineSensor6, kLineSensor7},
	NUM_SENSORS, TIMEOUT, kLineSensorLED); 
unsigned int sensorValues[NUM_SENSORS];

Servo m_frontRight;
Servo m_frontLeft;
Servo m_rearRight;
Servo m_rearLeft;

void setup()
{
	Serial.begin(115200);
	calibrate_qtrrc_sensor();
	m_frontRight.attach(kFrontRightMotor);
	m_frontLeft.attach(kFrontLeftMotor);
	m_rearRight.attach(kRearRightMotor);
	m_rearLeft.attach(kRearLeftMotor);
}

void loop()
{
	unsigned int position = read_position();
	track_line(position);
}

void calibrate_qtrrc_sensor()
{
	delay(500);
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
	for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
	{
		qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
	}
	digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

	// print the calibration minimum values measured when emitters were on
	Serial.begin(9600);
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	
	// print the calibration maximum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
	delay(1000);
}

unsigned int read_position()
{
	unsigned int position = qtrrc.readLine(sensorValues);
	return position;
}

void track_line(unsigned int position)
{
	Serial.print(position);
	Serial.print('\t');

	int accum = 0;
	for (int i = 0; i < NUM_SENSORS; i++) { accum += sensorValues[i]; }
	if (accum > 5000) { Serial.print("On a line.\t"); } //if more than 5 sensors are tripped

	int adjustedPosition = position - 3500;
	int leftDrive = (20 - (adjustedPosition * kLineTrackingP)) + 90;
	int rightDrive = (20 + (adjustedPosition * kLineTrackingP)) + 90;
	tankDrive(leftDrive, rightDrive);
}

void tankDrive(int leftSpeed, int rightSpeed)
{
	(leftSpeed < 0) ? leftSpeed = 0 : leftSpeed = leftSpeed;
	(rightSpeed < 0) ? rightSpeed = 0 : rightSpeed = rightSpeed;
	m_frontRight.write(rightSpeed);
	m_frontLeft.write(180 - leftSpeed);
	m_rearLeft.write(180 - leftSpeed);
	m_rearRight.write(rightSpeed);
	Serial.print(leftSpeed);
	Serial.print('\t');
	Serial.print(rightSpeed);
	Serial.println();
}
