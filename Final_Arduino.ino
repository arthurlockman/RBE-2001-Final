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

Direction StartingDirection = kNorth;
Direction currentDirection;

int leftdrive = 90;
int rightdrive = 90;
int claw_value = 0;
int conveyer_value = 90;
int FourBar_value = 0;

PPM ppm(2);

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);
	pinMode(kCompassPower, OUTPUT);

	// calibrate_qtrrc_sensor(); //THIS NEEDS TO BE MOVED
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

	homeConveyor();
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
		Serial.println("Robot initialized. Waiting for bluetooth...");
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
		leftdrive = ppm.getChannel(2);
		rightdrive = ppm.getChannel(3);

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

		conveyer_value = ppm.getChannel(6);

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
		m_conveyor.write(180 - conveyer_value);
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
	/*
	Serial.print(leftSpeed);
	Serial.print('\t');
	Serial.print(rightSpeed);
	Serial.println();
	*/
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

void turn(int angle)
{
	// lcd.clear();
	// lcd.setCursor(0,1);
	m_compass.read();
	float initial_heading = m_compass.heading();
	float desired_heading = (float)angle + initial_heading;
	if (desired_heading > 360.0) desired_heading -= 360.0;
	else if (desired_heading < 0.0) desired_heading += 360.0;
	char tmp[16];
	sprintf(tmp, "%d %d %d", angle, (int)desired_heading, (int)initial_heading);
	// lcd.print(tmp);
	while (abs(m_compass.heading() - desired_heading) > 2.5)
	{
		int multiplier = angle < 0 ? -1:1;
		int motorSpeed = multiplier * 30;
		tankDrive(motorSpeed, -motorSpeed);
		m_compass.read();
		// lcd.setCursor(1,1);
		// lcd.write(m_compass.heading());
	}
	stopDrive();
}
/*
void turnAround()
{
	// Turn until off of line
	while(read_position() < 6500)
	{
		turn(7);
	}

	// Turn until at 3500
	while(WithinTolerance(read_position(),3500, 100))
	{
		turn(7);
	}
}
*/



