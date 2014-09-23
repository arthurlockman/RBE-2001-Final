#ifndef SWAGDRIVE_H
#define SWAGDRIVE_H

#include "Servo.h"

class SwagDrive
{
public:
	SwagDrive(int right, int left);
	SwagDrive(int frontRight, int frontLeft, int rearRight, int rearLeft);
	void TankDrive(int leftSpeed, int rightSpeed);
private:
	int m_frontRightPort;
	int m_frontLeftPort;
	int m_rearRightPort;
	int m_rearLeftPort;
	Servo m_frontRight;
	Servo m_frontLeft;
	Servo m_rearRight;
	Servo m_rearLeft;
	int m_twoWheeled;
};

#endif
