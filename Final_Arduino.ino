#include "Robotmap.h"

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
unsigned int sensorAccum = 0;

//Motors
Servo m_frontRight;
Servo m_frontLeft;
Servo m_rearRight;
Servo m_rearLeft;
Servo m_claw;
Servo m_lineup;
Servo m_conveyor;
LiquidCrystal lcd(40, 41, 39, 38, 37, 36);

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
byte _radAlertHighPacket[10];
int _radAlertHighPacketSize;
byte _radAlertHighPacketData[3];
byte _radAlertLowPacket[10];
int _radAlertLowPacketSize;
byte _radAlertLowPacketData[3];

BTPacket kPktRadiationAlertHigh;
BTPacket kPktRadiationAlertLow;
BTPacket kPktHeartbeat;

int _sendhb = 0;

int leftdrive = 90;
int rightdrive = 90;
int claw_value = 0;
int conveyor_value = 90;
int FourBar_value = 0;

int m_autonomousStage = 0;
int m_autonomousTime = 0;

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);

	m_frontRight.attach(kFrontRightMotor);
	m_frontLeft.attach(kFrontLeftMotor);
	m_rearRight.attach(kRearRightMotor);
	m_rearLeft.attach(kRearLeftMotor);
	m_claw.attach(kClawMotor);
	m_lineup.attach(kFourBarMotor, 400, 2400);
	setFourBar(false);

	m_conveyor.attach(kConveyerMotor);
	
	Timer3.initialize(kTimerPeriod);
	Timer3.attachInterrupt(periodicUpdate);

	m_reactor.setDst(0x00);
	_heartbeatSize = m_reactor.createPkt(kBTHeartbeat, _heartbeatData, _heartbeatPacket);
	_radAlertHighPacketData[0] = kRadiationCarryingNew;
	_radAlertHighPacketSize = m_reactor.createPkt(kBTRadiationAlert, 
		_radAlertHighPacketData, _radAlertHighPacket);
	_radAlertLowPacketData[0] = kRadiationCarryingSpent;
	_radAlertLowPacketSize = m_reactor.createPkt(kBTRadiationAlert, 
		_radAlertLowPacketData, _radAlertLowPacket);

	kPktHeartbeat.packetData = _heartbeatPacket;
	kPktHeartbeat.packetSize = _heartbeatSize;
	kPktRadiationAlertHigh.packetData = _radAlertHighPacket;
	kPktRadiationAlertHigh.packetSize = _radAlertHighPacketSize;
	kPktRadiationAlertLow.packetData = _radAlertLowPacket;
	kPktRadiationAlertLow.packetSize = _radAlertLowPacketSize;

	lcd.begin(16, 2);

	pinMode(kStartButtonInput, INPUT_PULLUP);
}

void loop()
{
	byte _incomingPacket[10];
	byte _incomingType;
	byte _incomingData[3];
	if (m_btMaster.readPacket(_incomingPacket))
	{
		if (m_reactor.getData(_incomingPacket, _incomingData, _incomingType) && 
			(_incomingPacket[4] == kAddressRobot || _incomingPacket[4] == kAddressFMS))
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

	switch (kCurrentRobotState)
	{
	case kStartup:
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Calib. line...");
		calibrateLineSensor();
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Homing conveyor.");
		homeConveyor();
		openGripper();
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
			changeState(kAutonomous);
			lcd.clear();
			lcd.setCursor(0,0);
		}
		break;
	case kTankDrive:
		// leftdrive = ppm.getChannel(3);
		// rightdrive = ppm.getChannel(2);

		// if (abs(ppm.getChannel(5) - claw_value) > 150)
		// {
		// 	if (ppm.getChannel(5) < 90)
		// 	{
		// 		claw_value = 0;
		// 	}
		// 	else
		// 	{
		// 		claw_value = 180;
		// 	}
		// }

		// conveyor_value = ppm.getChannel(6);

		// if (abs(ppm.getChannel(4) - 90) > 30)
		// {
		// 	if (ppm.getChannel(4) < 90)
		// 	{
		// 		FourBar_value = 0;
		// 	}
		// 	else
		// 	{
		// 		FourBar_value = 80;
		// 	}
		// }

		// m_frontLeft.write(leftdrive);
		// m_rearLeft.write(leftdrive);
		// m_frontRight.write(180 - rightdrive);
		// m_rearRight.write(180 - rightdrive);
		// m_claw.write(claw_value);
		// if (abs(conveyor_value - 90) > 30)
		// {
		// 	if (conveyor_value < 90)
		// 	{
		// 		driveConveyor(kConveyorDown);
		// 	}
		// 	else
		// 	{
		// 		driveConveyor(kConveyorHome);
		// 	}
		// }
		// m_lineup.write(FourBar_value);
		// if (!digitalRead(kStartButtonInput))
		// {
		// 	stopDrive();
		// 	changeState(kAutonomous);
		// }
		break;
	case kAutonomous:		
		if (m_autonomousStage == 0)
		{
			lcd.clear();
			char tubeAvailData[16];
			snprintf(tubeAvailData, 16, "%d %d  %d %d", m_storageTubes.tubeOne, m_storageTubes.tubeTwo, 
				m_storageTubes.tubeThree, m_storageTubes.tubeFour);
			char tubeSupplyData[16];
			snprintf(tubeSupplyData, 16, "%d %d  %d %d", m_supplyTubes.tubeOne, m_supplyTubes.tubeTwo, 
				m_supplyTubes.tubeThree, m_supplyTubes.tubeFour);
			lcd.setCursor(0,0);
			lcd.print(tubeSupplyData);
			lcd.setCursor(0,1);
			lcd.print(tubeAvailData);
		}
		switch (m_autonomousStage)
		{
		case 0:
			driveConveyor(kConveyorDown);
			tankDrive(10,10);
			delay(100);
			m_autonomousStage++;
			break;
		case 1:
			closeGripper();
			delay(200);
			m_autonomousStage++;
			break;
		case 2:
			driveConveyor(kConveyorInsert);
			stopDrive();
			delay(100);
			m_autonomousStage++;
			break;
		case 3:
			turnAround(kTurnRight, 2);
			m_autonomousStage++;
			break;
		case 4:
			trackToLine();
			m_autonomousStage++;
			break;
		case 5:
			turn(kTurnRight);
			m_autonomousStage++;
			break;
		case 6:
			trackToLine();
			delay(500);
			m_autonomousStage++;
			break;
		case 7:
			setFourBar(true);
			delay(500);
			m_autonomousStage++;
			break;
		case 8:
			trackLineT(500);
			stopDrive();
			m_autonomousStage++;
			break;
		case 9:
			openGripper();
			delay(200);
			driveConveyor(kConveyorHome);
			delay(200);
			closeGripper();
			delay(200);
			driveConveyor(kConveyorInsert);
			setFourBar(false);
			delay(200);
			m_autonomousStage++;
			break;
		case 10:
			turnAround(kTurnLeft, 2);
			m_autonomousStage++;
			break;
		default:
			changeState(kDone);
			break;
		}
		break;
	case kPaused:
		Serial.println("Robot paused.");
		delay(100);
		break;
	case kDone:
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Done!");
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

void closeGripper()
{
	setGripper(true);
}

void openGripper()
{
	setGripper(false);
}

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

void setFourBar(bool out)
{
	if(out)
	{
		m_lineup.write(50);
	}
	else
	{
		m_lineup.write(0);
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
	noInterrupts();
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
	interrupts();
}

/**
 * @brief Send the heartbat message.
 * @details Sends the heartbeat message to the field computer.
 */
void sendHeartbeat()
{
	m_packetQueue.push(kPktHeartbeat);
}

/**
 * @brief Sends a robot movement status message.
 * @details Sends a message to the FMS detailing the robot's
 * current state.
 * 
 * @param mvt The movement status.
 * @param grp The gripper status.
 * @param opr The operation status.
 */
void sendStatusMessage(byte mvt, byte grp, byte opr)
{
	byte pkt[10];
	int  sz;
	byte data[3];
	data[0] = mvt;
	data[1] = grp;
	data[2] = opr;
	sz = m_reactor.createPkt(kBTRobotStatus, data, pkt);
	BTPacket btTmp = {pkt, sz};
	m_packetQueue.push(btTmp);
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

void calibrateLineSensor()
{
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

unsigned int readLinePosition()
{
	unsigned int position = m_lineSensor.readLine(sensorValues);
	sensorAccum = 0;
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		sensorAccum += sensorValues[i];
	}
	return position;
}

void trackToLine()
{
	unsigned int pos = readLinePosition();
	int strtTime = millis();
	while (millis() - strtTime < 300)
	{ 
		pos = readLinePosition();
		trackLine(pos);
	}
	while (sensorAccum < 6000)
	{
		pos = readLinePosition();
		trackLine(pos);
	}
	stopDrive();
}

void trackToLineReverse()
{
	unsigned int pos = readLinePosition();
	int strtTime = millis();
	while (millis() - strtTime < 300)
	{ 
		pos = readLinePosition();
		trackLineReverse(pos);
	}
	while (sensorAccum < 6000)
	{
		pos = readLinePosition();
		trackLineReverse(pos);
	}
	stopDrive();
}

void trackLine(unsigned int position)
{
	int adjustedPosition = position - 3500;
	int leftDrive = (15 - (adjustedPosition * kLineTrackingP));
	int rightDrive = (15 + (adjustedPosition * kLineTrackingP));
	tankDrive(leftDrive, rightDrive);
}

void trackLineT(int time)
{
	int sTime = millis();
	while (millis() - sTime < 500)
	{
		trackLine(readLinePosition());
	}
}

void trackLineReverse(unsigned int position)
{
	int adjustedPosition = position - 3500;
	int leftDrive = (- 15 - (adjustedPosition * kLineTrackingP));
	int rightDrive = (- 15 + (adjustedPosition * kLineTrackingP));
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
		while (millis() - intTime < 500) { trackLine(readLinePosition()); }
		while (sensorValues[2] < 500) 
		{ 
			readLinePosition();
			tankDrive(15, -15); 
		}
		stopDrive();
		break;
	case kTurnLeft:
		while (millis() - intTime < 500) { trackLine(readLinePosition()); }
		while (sensorValues[6] < 300) 
		{ 
			readLinePosition();
			tankDrive(-15, 15); 
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
		while(millis() - intTime < 400) { tankDrive(-20, -20); }
		stopDrive();
		for (int i = 0; i < expectedLines; i++)
		{
			while (sensorValues[2] < 500)
			{ 
				readLinePosition();
				tankDrive(15, -15); 
			}
			while (sensorValues[2] > 450)
			{ 
				readLinePosition();
				tankDrive(15, -15); 
			}
		}
		stopDrive();
		break;
	case kTurnLeft:
		while(millis() - intTime < 400) { tankDrive(-20, -20); }
		stopDrive();
		for (int i = 0; i < expectedLines; i++)
		{
			while (sensorValues[6] < 500)
			{ 
				readLinePosition();
				tankDrive(-15, 15); 
			}
			while (sensorValues[6] > 450)
			{ 
				readLinePosition();
				tankDrive(-15, 15); 
			}
		}
		stopDrive();
		break;
	}
}
