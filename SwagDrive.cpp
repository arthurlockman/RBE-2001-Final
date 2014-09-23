#include "SwagDrive.h"

SwagDrive::SwagDrive(int right, int left):
	m_twoWheeled(1),
	m_frontRightPort(right),
	m_frontLeftPort(left)
{
	m_frontRight.attach(m_frontRightPort);
	m_frontLeft.attach(m_frontLeftPort);
}

SwagDrive::SwagDrive(int frontRight, int frontLeft, int rearRight, int rearLeft):
	m_frontRightPort(frontRight),
	m_frontLeftPort(frontLeft),
	m_rearRightPort(rearRight),
	m_rearLeftPort(rearLeft),
	m_twoWheeled(0)
{
	m_frontRight.attach(m_frontRightPort);
	m_frontLeft.attach(m_frontLeftPort);
	m_rearRight.attach(m_rearRightPort);
	m_rearLeft.attach(m_rearLeftPort);
}

void SwagDrive::TankDrive(int leftSpeed, int rightSpeed)
{
	//@TODO: Fix these calculations
	int left = leftSpeed + 90;
	int right = 180 - (rightSpeed + 90);
	m_frontRight.write(right);
	m_frontLeft.write(left);
	if (!m_twoWheeled)
	{
		m_rearLeft.write(left);
		m_rearRight.write(right);
	}
}
