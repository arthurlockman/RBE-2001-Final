#include "Robotmap.h"
#include <QTRSensors.h>

//Alarm radiationAlarm(kRadiationAlarm);

#define NUM_SENSORS 8
#define TIMEOUT 2500  

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {kLineSensor0, kLineSensor1, kLineSensor2, 
	kLineSensor3, kLineSensor4, kLineSensor5, kLineSensor6, kLineSensor7},
	NUM_SENSORS, TIMEOUT, kLineSensorLED); 
unsigned int sensorValues[NUM_SENSORS];

//Motors
Servo m_frontRight;
Servo m_frontLeft;
Servo m_rearRight;
Servo m_rearLeft;
Servo m_claw;
Servo m_lineup;
Servo m_conveyor;

//ISR Variables
volatile long interruptCount = 0;
static const int kTimerPeriod = 50000;

void setup()
{
	Serial.begin(115200);
	calibrate_qtrrc_sensor();
	m_frontRight.attach(kFrontRightMotor);
	m_frontLeft.attach(kFrontLeftMotor);
	m_rearRight.attach(kRearRightMotor);
	m_rearLeft.attach(kRearLeftMotor);
	m_claw.attach(kClawMotor);
	m_lineup.attach(kFourBarMotor);
	m_conveyor.attach(kConveyerMotor);
	Timer3.initialize(kTimerPeriod);
	Timer3.attachInterrupt(periodicUpdate);
}

void loop()
{
	unsigned int position = read_position();
	track_line(position);
}

/**
 * @brief Performs tasks that need to be updated
 * on a fixed period.
 * @details Performs tasks that need to be done 
 * at fixed time intervals. Checks the number of interrupts
 * and takes the modulus to figure out if a certain
 * task should be done at that time or not.
 */
void periodicUpdate()
{
	//Do things here with interruptcount.
	interruptCount++;
}

void calibrate_qtrrc_sensor()
{
	delay(500);
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	for (int i = 0; i < 400; i++)
	{
		qtrrc.calibrate();
	}
	digitalWrite(13, LOW);

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

void track_to_line()
{
	int foundLine = 0;
	while (!foundLine)
	{
		unsigned int position = read_position();
		track_line(position);
		int accum = 0;
		for (int i = 0; i < NUM_SENSORS; i++) { accum += sensorValues[i]; }
		if (accum > 5000) 
		{ 
			Serial.print("On a line. Stopping.\t"); 
			foundLine = 1;
		}
	}
}

void track_line(unsigned int position)
{
	Serial.print(position);
	Serial.print('\t');
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

void stopDrive()
{
	m_frontRight.write(90);
	m_frontLeft.write(90);
	m_rearLeft.write(90);
	m_rearRight.write(90);
}

/**
 * @brief Stop all motors
 * @details This method stops all motors on the robot.
 */
void stopAll()
{
	m_frontRight.write(90);
	m_frontLeft.write(90);
	m_rearRight.write(90);
	m_rearLeft.write(90);
	m_conveyor.write(90);
	m_claw.write(90);
	m_lineup.write(90);
}