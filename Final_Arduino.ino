#include "Robotmap.h"

//Alarm radiationAlarm(kRadiationAlarm);

#define NUM_SENSORS 8
#define TIMEOUT 2500

QTRSensorsRC m_lineSensor((unsigned char[])
{
	kLineSensor0, kLineSensor1, kLineSensor2, 
	kLineSensor3, kLineSensor4, kLineSensor5, 
	kLineSensor6, kLineSensor7
},
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
LiquidCrystal lcd(40, 41, 39, 38, 37, 36);

LSM303 m_compass;
Encoder m_trackEncoder(kEncoder1, kEncoder2);
BumpSensor m_bumpSensor(kAccX, kAccY, kAccZ, 50);

ReactorProtocol m_reactor(kAddressRobot);
BluetoothMaster m_btMaster;

QueueList<BTPacket> m_packetQueue;

volatile unsigned long interruptCount = 0;
static const unsigned long kTimerPeriod = 100000;
static const unsigned long kSecond = 1000000;
static const unsigned long kHeartbeatPeriod =  kSecond / kTimerPeriod;

int kCurrentRobotState = kStartup;
int kPreviousRobotState = kStartup;
TubeAvailability m_supplyTubes;
TubeAvailability m_storageTubes;

byte _heartbeatPacket[10];
int  _heartbeatSize;
byte _heartbeatData[3];
int _sendhb = 0;

int leftdrive = 90;
int rightdrive = 90;
int claw_value = 0;
int conveyor_value = 90;
int FourBar_value = 0;

PPM ppm(2);

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);
	pinMode(kCompassPower, OUTPUT);

	calibrate_qtrrc_sensor(); //THIS NEEDS TO BE MOVED
	m_frontRight.attach(kFrontRightMotor);
	m_frontLeft.attach(kFrontLeftMotor);
	m_rearRight.attach(kRearRightMotor);
	m_rearLeft.attach(kRearLeftMotor);
	m_claw.attach(kClawMotor);
	//m_lineup.attach(kFourBarMotor);
	
	m_conveyor.attach(kConveyerMotor);
	Timer3.initialize(kTimerPeriod);
	Timer3.attachInterrupt(periodicUpdate);

	m_reactor.setDst(0x00);
	_heartbeatSize = m_reactor.createPkt(kBTHeartbeat, _heartbeatData, _heartbeatPacket);

	lcd.begin(16, 2);

	turnAround(kTurnRight, 2);
	track_to_line();
	turn(kTurnRight);
	track_to_line();
	turnAround(kTurnRight, 2);
	track_to_line();
	int initTime = millis();
	while (millis() - initTime < 250) { tankDrive(30, 30); }
	track_to_line();

	pinMode(kStartButtonInput, INPUT_PULLUP);
}

void loop()
{
	byte _incomingPacket[10];
	byte _incomingType;
	byte _incomingData[3];
	if (m_btMaster.readPacket(_incomingPacket))
	{
		if (m_reactor.getData(_incomingPacket, _incomingData, _incomingType))
		{
			switch (_incomingType)
			{
			case kBTStorageTubeAvailable:
				updateAvailablity(_incomingData[0], &m_storageTubes);
				break;
			case kBTSupplyTubeAvailable:
				updateAvailablity(_incomingData[0], &m_supplyTubes);
				break;
			case kBTStopMovement:
				changeState(kPaused);
				break;
			case kBTResumeMovement:
				revertState();
				break;
			}
		}
	}

	//unsigned int position = read_position();
	//track_line(position);
	switch (kCurrentRobotState)
	{
	case kStartup:
		Serial.println("Homing conveyor...");
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Homing conveyor.");
		homeConveyor();
		Serial.println("Robot initialized.");
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Wait for BT...");
		changeState(kWaitForBluetooth);
		break;
	case kWaitForBluetooth:
		if (!digitalRead(kStartButtonInput) && _sendhb == 0)
		{
			_sendhb = 1;
			Serial.println("Sending heartbeat...");
			changeState(kTankDrive);
		}
		break;
	case kTankDrive:
		leftdrive = ppm.getChannel(3);
		rightdrive = ppm.getChannel(2);

		if (abs(ppm.getChannel(5) - claw_value) > 150)
		{
			if (ppm.getChannel(5) < 90)
			{
				claw_value = 0;
			}
			else
			{
				claw_value = 180;
			}
		}

		conveyor_value = ppm.getChannel(6);

		if (abs(ppm.getChannel(4) - 90) > 30)
		{
			if (ppm.getChannel(4) < 90)
			{
				FourBar_value = 0;
			}
			else
			{
				FourBar_value = 80;
			}
		}

		m_frontLeft.write(leftdrive);
		m_rearLeft.write(leftdrive);
		m_frontRight.write(180 - rightdrive);
		m_rearRight.write(180 - rightdrive);
		m_claw.write(claw_value);
		if (abs(conveyor_value - 90) > 30)
		{
			if (conveyor_value < 90)
			{
				driveConveyor(kConveyorDown);
			}
			else
			{
				driveConveyor(kConveyorHome);
			}
		}
		m_lineup.write(FourBar_value);
		break;
	}
}

void homeConveyor()
{
	if (!digitalRead(kConveyorLimit)) return;
	while (digitalRead(kConveyorLimit))
	{
		m_conveyor.write(120);
	}
	m_trackEncoder.write(0);
	m_conveyor.write(90);
}

/**
 * @brief Drive the conveyor to a specified position.
 * @details Drive the conveyor to a specfied position.
 * This doesn't drive to a relative position.
 * 
 * @param position The position to drive to.
 */
void driveConveyor(int position)
{
	long pos = m_trackEncoder.read();
	if (pos == position) return;
	int speed = (pos < position)? 60 : 120;
	while (abs(pos - position) > 1)
	{
		m_conveyor.write(speed);
		pos = m_trackEncoder.read();
	}
	m_conveyor.write(180 - speed);
	m_conveyor.write(90);
}

void autonomous()
{
	// Grab Tube
		// Lower Conveyer

		// Grab tube

		// Raise Conveyer
		//grabLowerTube();

	// Turn around

	// Drive to open tube
		// Determine open tube

		// Drive to line

		// Turn right

		// Drive to stop

	// Insert tube

	// Turn around

	// Drive to middle

	// Drive to filled reactor
		// Determine filled reactor

		// Turn (or not)

		// Drive to stop

	// Grab tube

	// Turn around

	// Return to middle

	// Turn right

	// Drive to stop

	// Insert tube

	// End


}

/*
void grabLowerTube()
{
	setConveyerPosition(DOWN);
	setGripper(true);
	setConveyerPosition(HOME);
}

// Send the conveyer to some position
void setConveyerPosition(DATA pos)
{

}
*/

// Sets the gripper
// true -> closed
// false -> open
void setGripper(bool closed)
{
	if(closed)
	{
		m_claw.write(180);
	}
	else
	{
		m_claw.write(0);
	}
}

/**
 * @brief Changes robot state.
 * @details Changes the current robot state
 * and stores the previous one.
 * 
 * @param newState The new state to change to.
 */
void changeState(int newState)
{
	kPreviousRobotState = kCurrentRobotState;
	kCurrentRobotState = newState;
	Serial.print("State: ");
	Serial.println(newState);
}

/**
 * @brief Revert robot state.
 * @details This method reverts the robot state
 * to whatever it was before it was most recently changed.
 */
void revertState()
{
	int _tmp = kPreviousRobotState;
	kPreviousRobotState = kCurrentRobotState;
	kCurrentRobotState = _tmp;
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
	if (_sendhb && (interruptCount % kHeartbeatPeriod) == 0)
	{
		sendHeartbeat();
	}
	if (!m_packetQueue.isEmpty())
	{
		BTPacket p = m_packetQueue.pop();
		m_btMaster.sendPkt(p.packetData, p.packetSize);
	}
}

/**
 * @brief Send the heartbat message.
 * @details Sends the heartbeat message to the field computer.
 */
void sendHeartbeat()
{
	BTPacket tmp = {_heartbeatPacket, _heartbeatSize};
	m_packetQueue.push(tmp);
	sendMessage(0x00, 0x03, 0x2C);
}

/**
 * @brief Sends a bluetooth message.
 * @details Creates a bluetooth message packet and adds it
 * to the message queue. It will be sent out when appropriate
 * by the ISR.
 * 
 * @param destination The destination address.
 * @param type The message type.
 * @param data The message data.
 */
void sendMessage(byte destination, byte type, byte data)
{
	m_reactor.setDst(destination);
	byte tmpData[3];
	tmpData[0] = data;
	byte pkt[10];
	int size = m_reactor.createPkt(type, tmpData, pkt);
	BTPacket packet = {pkt, size};
	m_packetQueue.push(packet);
}

/**
 * @brief Updates availability of storage tubes.
 * @details Update the availability of storage tubes
 * stored in memory.
 * 
 * @param data The incoming data.
 * @param storage The destination structure to put
 * storage data in.
 */
void updateAvailablity(byte data, TubeAvailability* storage)
{
	storage->tubeOne = bitRead(data, 0);
	storage->tubeTwo = bitRead(data, 1);
	storage->tubeThree = bitRead(data, 2);
	storage->tubeFour = bitRead(data, 3);
}

void calibrate_qtrrc_sensor()
{
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Calib. line sens");
	delay(500);
	for (int i = 0; i < 400; i++)
	{
		m_lineSensor.calibrate();
	}

	// print the calibration minimum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(m_lineSensor.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();

	// print the calibration maximum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(m_lineSensor.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
	delay(1000);
}

unsigned int read_position()
{
	unsigned int position = m_lineSensor.readLine(sensorValues);
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
		for (int i = 0; i < NUM_SENSORS; i++)
		{
			accum += sensorValues[i];
		}
		if (accum > 5000)
		{
			Serial.print("On a line. Stopping.\t");
			foundLine = 1;
			stopDrive();
		}
	}
}

void track_line(unsigned int position)
{
	Serial.println(position);
	int adjustedPosition = position - 3500;
	int leftDrive = (20 - (adjustedPosition * kLineTrackingP));
	int rightDrive = (20 + (adjustedPosition * kLineTrackingP));
	tankDrive(leftDrive, rightDrive);
}

void tankDrive(int leftSpeed, int rightSpeed)
{
	leftSpeed = leftSpeed + 90;
	rightSpeed = rightSpeed + 90;
	(leftSpeed < 0) ? leftSpeed = 0 : (leftSpeed > 180) ? leftSpeed = 180 : leftSpeed = leftSpeed;
	(rightSpeed < 0) ? rightSpeed = 0 : (rightSpeed > 180) ? rightSpeed = 180 : rightSpeed = rightSpeed;
	m_frontRight.write(rightSpeed);
	m_frontLeft.write(180 - leftSpeed);
	m_rearLeft.write(180 - leftSpeed);
	m_rearRight.write(rightSpeed);
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

/**
 * @brief Turns the robot a specified direction.
 * @details Turns the robot a direction based on 
 * the input direction. kTurn180 turns the robot 
 * around 180 degrees.
 * 
 * @param dir The direction to turn.
 */
void turn(int dir)
{
	int intTime = millis();
	switch (dir)
	{
	case kTurnRight:
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Turning right...");
		while (millis() - intTime < 500) { track_line(read_position()); }
		while (sensorValues[2] < 500) 
		{ 
			read_position();
			tankDrive(30, -30); 
		}
		stopDrive();
		break;
	case kTurnLeft:
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Turning left...");
		while (millis() - intTime < 500) { track_line(read_position()); }
		while (sensorValues[6] < 500) 
		{ 
			read_position();
			tankDrive(-30, 30); 
		}
		stopDrive();
		break;
	}
}

void turnAround(int dir, int expectedLines)
{
	int intTime = millis();
	switch (dir)
	{
	case kTurnRight:
		while(millis() - intTime < 250) { tankDrive(-30, -30); }
		stopDrive();
		for (int i = 0; i < expectedLines; i++)
		{
			while (sensorValues[2] < 500)
			{ 
				read_position();
				tankDrive(30, -30); 
			}
			while (sensorValues[2] > 450)
			{ 
				read_position();
				tankDrive(30, -30); 
			}
		}
		stopDrive();
		break;
	case kTurnLeft:
		while(millis() - intTime < 250) { tankDrive(-30, -30); }
		stopDrive();
		for (int i = 0; i < expectedLines; i++)
		{
			while (sensorValues[6] < 500)
			{ 
				read_position();
				tankDrive(-30, 30); 
			}
			while (sensorValues[6] > 450)
			{ 
				read_position();
				tankDrive(-30, 30); 
			}
		}
		stopDrive();
		break;
	}
}
