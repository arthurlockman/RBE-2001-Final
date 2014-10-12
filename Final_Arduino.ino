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

Servo m_frontRight;
Servo m_frontLeft;
Servo m_rearRight;
Servo m_rearLeft;
Servo m_claw;
Servo m_lineup;
Servo m_conveyor;
LiquidCrystal lcd(40, 41, 39, 38, 37, 36);

bool second_pass = false;
bool has_new_rod = false, has_old_rod = false; 

Encoder m_trackEncoder(kEncoder1, kEncoder2);

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
unsigned long m_autonomousTime = 0;
int m_autonomousLinesPassed = 0;
int m_autonomousLinesToPass = 0;
Direction m_autonomousNextDir = kTurnRight;
RobotPosition m_autonomousPosition = kReactorOne;

/**
 * @brief This method initializes all objects in
 * the program.
 * @details This method is the Arduino setup method.
 * It initializes everything that needs initializing. This 
 * method is only run once, at program startup.
 */
void setup()
{
	Serial.begin(115200); // Serial
	Serial1.begin(115200); // Bluetooth

	// Attach Motors
	m_frontRight.attach(kFrontRightMotor);
	m_frontLeft.attach(kFrontLeftMotor);
	m_rearRight.attach(kRearRightMotor);
	m_rearLeft.attach(kRearLeftMotor);
	m_claw.attach(kClawMotor);
	m_lineup.attach(kFourBarMotor, 400, 2400);
	setFourBar(false);

	m_conveyor.attach(kConveyerMotor);
	
	Timer3.initialize(kTimerPeriod);
	Timer3.attachInterrupt(periodicUpdate); // Periodically updates the bluetooth.

	// Bluetooth packet formation.
	// Forms the radiation and heartbeat packets.
	m_reactor.setDst(0x00);
	_heartbeatSize = m_reactor.createPkt(kBTHeartbeat, _heartbeatData, _heartbeatPacket);
	_radAlertHighPacketData[0] = kRadiationCarryingNew;
	_radAlertHighPacketSize = m_reactor.createPkt(kBTRadiationAlert, 
		_radAlertHighPacketData, _radAlertHighPacket);
	_radAlertLowPacketData[0] = kRadiationCarryingSpent;
	_radAlertLowPacketSize = m_reactor.createPkt(kBTRadiationAlert, 
		_radAlertLowPacketData, _radAlertLowPacket);

	// Base information for storage and supply tubes. 
	m_storageTubes.tubeOne = 0;
	m_storageTubes.tubeTwo = 0;
	m_storageTubes.tubeThree = 0;
	m_storageTubes.tubeFour = 0;
	m_supplyTubes.tubeOne = 1;
	m_supplyTubes.tubeTwo = 1;
	m_supplyTubes.tubeThree = 1;
	m_supplyTubes.tubeFour = 1;

	// Bluetooth packet information.
	kPktHeartbeat.packetData = _heartbeatPacket;
	kPktHeartbeat.packetSize = _heartbeatSize;
	kPktRadiationAlertHigh.packetData = _radAlertHighPacket;
	kPktRadiationAlertHigh.packetSize = _radAlertHighPacketSize;
	kPktRadiationAlertLow.packetData = _radAlertLowPacket;
	kPktRadiationAlertLow.packetSize = _radAlertLowPacketSize;

	// Begin the lcd
	lcd.begin(16, 2);

	// Set the sensor pins.
	pinMode(kStartButtonInput, INPUT_PULLUP);
	pinMode(kPinStopLimit, INPUT_PULLUP);
	pinMode(kRadiationAlertHigh, OUTPUT);
	pinMode(kRadiationAlertLow, OUTPUT);
}

/**
 * @brief The main loop.
 * @details This function is the main arduino loop. It
 * is called every time it finishes and loops around.
 */
void loop()
{
	char tubeAvailData[16];
	char tubeSupplyData[16];
	byte _incomingPacket[10];
	byte _incomingType;
	byte _incomingData[3];
	// Read all of the incoming packets from the bluetooth master. 
	while (m_btMaster.readPacket(_incomingPacket))
	{
		Serial.println("Got packet.");
		_sendhb = 1;
		// Make sure packet is addressed to us. 
		if (m_reactor.getData(_incomingPacket, _incomingData, _incomingType) && 
			(_incomingPacket[4] == kAddressRobot || _incomingPacket[4] == kAddressFMS))
		{
			// Switch for which packet we recieved. 
			Serial.println("Got packet addressed to us.");
			switch (_incomingType)
			{
			case kBTStorageTubeAvailable:
				Serial.println("Got storage tube availability.");
				updateAvailablity(_incomingData[0], &m_storageTubes);
				break;
			case kBTSupplyTubeAvailable: 
				Serial.println("Got supply tube availablity.");
				updateAvailablity(_incomingData[0], &m_supplyTubes);
				break;
			case kBTStopMovement: // Case for robot stop.
				Serial.println("Got stop message.");
				changeState(kPaused);
				break;
			case kBTResumeMovement: // Case for robot resume.
				revertState();
				Serial.println("Got resume message.");
				break;
			default:
				Serial.println("Got unknown.");
				break;
			}
		}
	}
	switch (kCurrentRobotState)
	{
	case kStartup: // Case for calibrating sensors and preparing the robot to run. 
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
		lcd.print("Wait for start..");
		changeState(kWaitForBluetooth);
		break;
	case kWaitForBluetooth: // Waits for bluetooth to connect. Press button on top to start. 
		if (!digitalRead(kStartButtonInput))
		{
			Serial.println("Starting auto...");
			changeState(kBegin);
			lcd.clear();
			lcd.setCursor(0,0);
		}
		break;
	case kBegin: // Robot begins on center line. Tracks line until it encounters the reactor tube.
		trackLine(readLinePosition());
		if(digitalRead(kPinStopLimit) == LOW)
		{
			kCurrentRobotState = kAutonomous;
		}
		break;

	case kAutonomous: // Case for all of the autonomous code. 		
		if (m_autonomousStage == 0)
		{
			lcd.clear();
		}
		// Get supply and storage tube information.
		snprintf(tubeAvailData, 16, "%d %d  %d %d", m_storageTubes.tubeOne, m_storageTubes.tubeTwo, 
			m_storageTubes.tubeThree, m_storageTubes.tubeFour); 
		snprintf(tubeSupplyData, 16, "%d %d  %d %d", m_supplyTubes.tubeOne, m_supplyTubes.tubeTwo, 
			m_supplyTubes.tubeThree, m_supplyTubes.tubeFour);
		// Print supply and storage information to the lcd.
		lcd.setCursor(0,0);
		lcd.print(tubeSupplyData);
		lcd.setCursor(0,1);
		lcd.print(tubeAvailData);
		switch (m_autonomousStage)
		{
		case 0: //Drive conveyor down
			driveConveyor(kConveyorDown);
			tankDrive(10,10);
			delay(100);
			m_autonomousStage++;
			break;
		case 1: //Grab first tube
			closeGripper();
			delay(200);
			m_autonomousStage++;
			break;
		case 2: //Drive conveyor up
			has_old_rod = true;
			digitalWrite(kRadiationAlertLow, HIGH);
			driveConveyor(kConveyorInsert);
			stopDrive();
			delay(100);
			m_autonomousStage++;
			break;
		case 3: //Turn around to begin run
			turnAround(kTurnRight, 2);
			m_autonomousLinesPassed = 0;
			m_autonomousStage++;
			break;
		case 4://Drive to first storage tube
			if (!m_storageTubes.tubeFour)
			{
				trackToLine();
				m_autonomousStage++;
			} else if (!m_storageTubes.tubeThree) {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 2) { m_autonomousStage++; }
			} else if (!m_storageTubes.tubeTwo) {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 3) { m_autonomousStage++; }
			} else {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 4) { m_autonomousStage++; }
			}
			break;
		case 5: //Turn to enter first storage tube
			turn(kTurnRight);
			m_autonomousStage++;
			break;
		case 6: //Extend fourbar to line up with first storage tube
			setFourBar(true);
			delay(500);
			m_autonomousStage++;
			break;
		case 7: //Track line to stop before first storage
			trackLine(readLinePosition());
			if (sensorAccum > 5000) 
			{ 
				m_autonomousStage++; 
				m_autonomousTime = millis();
			}
			break;
		case 8: //Drive up to first storage.
			trackLine(readLinePosition());
			if (millis() - m_autonomousTime > 1000) 
			{ 
				trackLine(readLinePosition());
				m_autonomousStage++; 
			}
			break;
		case 9: //Release tube into first storage.
			openGripper();
			delay(200);
			driveConveyor(kConveyorHome);
			m_autonomousStage++;
			break;
		case 10: //Check if tube is inserted, if it isn't give it a tap.
			delay(500);
			if (m_autonomousLinesPassed == 0 && m_storageTubes.tubeFour) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 2 && m_storageTubes.tubeThree) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 3 && m_storageTubes.tubeTwo) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 4 && m_storageTubes.tubeOne) {
				m_autonomousStage++;
			} else {
				closeGripper();
				delay(200);
				driveConveyor(20);
				delay(500);
				m_autonomousStage++;
			}
			has_old_rod = false;
			digitalWrite(kRadiationAlertLow, LOW);
			break;
		case 11: //Retract fourbar and gripper to move to second reactor for pickup
			openGripper();
			driveConveyor(kConveyorHome);
			stopDrive();
			setFourBar(false);
			m_autonomousStage++;
			break;
		case 12: //Back away from first storage tube
			trackToLineReverse();
			m_autonomousStage++;
			break;
		case 13: //Turn around to face center
			if (m_autonomousLinesPassed == 2 || m_autonomousLinesPassed == 4)
				turnAround(kTurnLeft, 2);
			else 
				turnAround(kTurnRight, 2);
			m_autonomousStage++;
			break;
		case 14: //Track to center
			trackToLine();
			m_autonomousStage++;
			m_autonomousLinesPassed = 0;
			break;
		case 15: //Turn right to head to second reactor
			turn(kTurnRight);
			m_autonomousLinesToPass = 4 - m_autonomousLinesPassed;
			m_autonomousLinesPassed = 0;
			m_autonomousStage++;
			break;
		case 16: //Track line to second reactor stop line
			if (digitalRead(kPinStopLimit)) { trackLine(readLinePosition()); }
			else {m_autonomousStage++; }
			break;
		case 17: //Drive forward to engage second reactor.
			if (millis() - m_autonomousTime < 1500)
			{
				trackLine(readLinePosition());
			} else { 
				tankDrive(10, 10);
				m_autonomousStage++;
			}
			break;
		case 18: //Drive conveyor down to pickup from reactor 2.
			driveConveyor(kConveyorDown);
			tankDrive(10,10);
			delay(100);
			m_autonomousStage++;
			break;
		case 19: //Grab second reactor spent rod.
			closeGripper();
			delay(200);
			m_autonomousStage++;
			break;
		case 20: //Lift second reactor spent rod.
			has_old_rod = true;
			digitalWrite(kRadiationAlertLow, HIGH);
			driveConveyor(kConveyorInsert);
			stopDrive();
			delay(100);
			m_autonomousStage++;
			break;
		case 21: //Turn around to put second rod in storage.
			turnAround(kTurnRight, 2);
			m_autonomousLinesPassed = 0;
			m_autonomousStage++;
			break;
		case 22: //Determine which storage tube to use for second rod.
			if (!m_storageTubes.tubeOne)
			{
				trackToLine();
				m_autonomousStage++;
				m_autonomousLinesPassed++;
			} else if (!m_storageTubes.tubeTwo) {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 2) { m_autonomousStage++; }
			} else if (!m_storageTubes.tubeThree) {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 3) { m_autonomousStage++; }
			} else {
				trackToLine();
				m_autonomousLinesPassed++;
				if (m_autonomousLinesPassed == 4) { m_autonomousStage++; }
			}
			break;
		case 23: //Turn to face second storage tube.
			turn(kTurnLeft);
			m_autonomousStage++;
		case 24: //Extend fourbar to line up with second tube.
			setFourBar(true);
			delay(500);
			m_autonomousStage++;
			break;
		case 25: //Track line to second storage tube.
			trackLine(readLinePosition());
			if (sensorAccum > 5000) 
			{ 
				m_autonomousStage++; 
				m_autonomousTime = millis();
			}
			break;
		case 26: //Engage with second storage tube.
			trackLine(readLinePosition());
			if (millis() - m_autonomousTime > 1000) 
			{ 
				trackLine(readLinePosition());
				m_autonomousStage++; 
			}
			break;
		case 27: //Release second rod.
			openGripper();
			delay(200);
			driveConveyor(kConveyorHome);
			m_autonomousStage++;
			break;
		case 28: //If second rod not inserted, give it a tap.
			delay(500);
			if (m_autonomousLinesPassed == 1 && m_storageTubes.tubeOne) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 2 && m_storageTubes.tubeTwo) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 3 && m_storageTubes.tubeThree) {
				m_autonomousStage++;
			} else if (m_autonomousLinesPassed == 4 && m_storageTubes.tubeFour) {
				m_autonomousStage++;
			} else {
				closeGripper();
				delay(200);
				driveConveyor(20);
				delay(500);
				m_autonomousStage++;
			}
			has_old_rod = false;
			digitalWrite(kRadiationAlertLow, LOW);
			break;
		case 29: //Retract fourbar to head for new rods.
			openGripper();
			driveConveyor(kConveyorHome);
			stopDrive();
			setFourBar(false);
			m_autonomousStage++;
			break;
		case 30: //Back away from second storage tube.
			trackToLineReverse();
			m_autonomousStage++;
			break;
		case 31: //Turn to face center.
			if (m_autonomousLinesPassed == 2 || m_autonomousLinesPassed == 4)
				turnAround(kTurnRight, 2);
			else 
				turnAround(kTurnLeft, 2);
			m_autonomousStage++;
			break;
		case 32: //Track to center to head for reactor.
			trackToLine();
			m_autonomousStage++;
			break;
		case 33: //Determine which supply tube to go for first.
			if (m_autonomousLinesPassed == 1) //if already at tube 1
			{
				m_autonomousPosition = kTubeOne;
				if (m_supplyTubes.tubeOne)
				{
					m_autonomousStage = 36;
					break;
				} else {
					turn(kTurnLeft);
					m_autonomousStage++;
					m_autonomousNextDir = kTurnRight;
				}
			} else if (m_autonomousLinesPassed == 2) { //if at tube 2
				m_autonomousPosition = kTubeTwo;
				if (m_supplyTubes.tubeTwo)
				{
					m_autonomousStage = 36;
				} else if (!m_supplyTubes.tubeOne) {
					turn(kTurnLeft);
					m_autonomousNextDir = kTurnRight;
					m_autonomousStage++;
				} else {
					turn(kTurnRight);
					m_autonomousNextDir = kTurnLeft;
					m_autonomousStage++;
				}
			} else if (m_autonomousLinesPassed == 3) { //if at tube 3
				m_autonomousPosition = kTubeThree;
				if (m_supplyTubes.tubeThree)
				{
					m_autonomousStage = 36;
				} else if (!m_supplyTubes.tubeFour) {
					m_autonomousNextDir = kTurnLeft;
					turn(kTurnRight);
					m_autonomousStage++;
				} else {
					turn(kTurnLeft);
					m_autonomousNextDir = kTurnRight;
					m_autonomousStage++;
				}
			} else if (m_autonomousLinesPassed == 4) { //if at tube 4
				m_autonomousPosition == kTubeFour;
				if (m_supplyTubes.tubeFour)
				{
					m_autonomousStage = 36;
				} else {
					m_autonomousStage++;
					turn(kTurnRight);
					m_autonomousNextDir = kTurnLeft;
				}
			} else {turn(kTurnRight);}
			m_autonomousLinesPassed = 0;
			m_autonomousLinesToPass = supplyLinesToCross(m_autonomousPosition);
			break;
		case 34: //Drive to supply tube crossing
			trackToLine();
			m_autonomousLinesPassed++;
			if (m_autonomousLinesPassed == m_autonomousLinesToPass) { m_autonomousStage++; }
			break;
		case 35: //Turn to face supply tube
			turn(m_autonomousNextDir);
			m_autonomousStage++;
			break;
		case 36: //Drive to supply tube.
			trackToLine();
			m_autonomousStage++;
			break;
		case 37: // Extend the four bar and drive up to the tube.
			setFourBar(true);
			delay(600);
			m_autonomousTime = millis();
			while (millis() - m_autonomousTime < 500) { trackLine(readLinePosition()); }
			m_autonomousStage++;
			break;
		case 38: // Drive the claw to the remove position
			driveConveyor(kConveyorRemove);
			m_autonomousStage++;
			break;
		case 39: // Grab the rod and extract it
			closeGripper();
			delay(200);
			driveConveyor(kConveyorHome);
			has_new_rod = true;
			digitalWrite(kRadiationAlertHigh, HIGH);
			m_autonomousStage++;
			break;
		case 40: // Retract the four bar, back up, and turn around.
			setFourBar(false);
			delay(200);
			trackToLineReverse();
			delay(300);
			turnAround(kTurnRight, 2);
			m_autonomousStage++;
			break;
		case 41: // Track to the middle of the field.
			trackToLine();
			m_autonomousStage++;
			break;
		case 42: // Turn left if it's the first supply rod, else right.
			if(!second_pass)
			{turn(kTurnLeft);}
			else
			{turn(kTurnRight);}
			m_autonomousStage++;
			break;
		case 43: // Track line to the reactor tube.
			trackLine(readLinePosition());
			if(digitalRead(kPinStopLimit) == LOW)
			{
				m_autonomousStage++;
			}
			break;
		case 44: // Insert the rod into the tube. 
			tankDrive(10,10);
			driveConveyor(kConveyorDown);
			delay(200);
			openGripper();
			delay(200);
			driveConveyor(kConveyorHome);
			has_new_rod = false;
			digitalWrite(kRadiationAlertHigh, LOW);
			m_autonomousStage++;
			break;
		case 45: // Turn around. Stop if done with all rods.
			turnAround(kTurnRight, 2);
			if(!second_pass)
			{m_autonomousStage++;}
			else
			{m_autonomousStage = 100;}
			break;
		case 46: // Find the line for the next supply rod.
			if(m_supplyTubes.tubeFour)
			{
				m_autonomousLinesToPass = 4;
			}
			else if(m_supplyTubes.tubeThree)
			{
				m_autonomousLinesToPass = 3;
			}
			else if(m_supplyTubes.tubeTwo)
			{
				m_autonomousLinesToPass = 2;
			}
			else if(m_supplyTubes.tubeOne)
			{
				m_autonomousLinesToPass = 1;
			}
			m_autonomousLinesPassed = 0;
			m_autonomousStage++;
			break;
		case 47: // Track to the line of the next supply tube. Loop to step 35. 
			trackToLine();
			m_autonomousLinesPassed++;
			if(m_autonomousLinesPassed == m_autonomousLinesToPass)
			{
				m_autonomousNextDir = kTurnRight;
				second_pass = true;
				m_autonomousStage = 35;
			}
			break;
		default: //Complete!
			changeState(kDone);
			lcd.clear();
			break;
		}
		break;
	case kPaused: // Pause command sent from master bluetooth. 
		Serial.println("Robot paused.");
		stopDrive();
		delay(100);
		break;
	case kDone: // Robot has completed all tasks. 
		lcd.setCursor(0,0);
		lcd.print("Done!");
		break;
	}
}

/**
 * @brief Number of lines to cross to reach desired
 * supply tube.
 * @details This method determines the number of lines the robot needs
 * to cross to reach the desired supply tube. It takes in the current
 * position and returns a number.
 * 
 * @param startPos The starting RobotPosition.
 * @return The number of lines to cross to get to the next
 * open tube.
 */
int supplyLinesToCross(RobotPosition startPos)
{
	switch (startPos)
	{
	case kTubeOne:
		if (m_supplyTubes.tubeTwo)
			return 1;
		else if (m_supplyTubes.tubeThree)
			return 2;
		else if (m_supplyTubes.tubeFour)
			return 3;
		break;
	case kTubeTwo:
		if (m_supplyTubes.tubeThree || m_supplyTubes.tubeOne)
			return 1;
		else if (m_supplyTubes.tubeFour)
			return 2;
		break;
	case kTubeThree:
		if (m_supplyTubes.tubeFour || m_supplyTubes.tubeTwo)
			return 1;
		else if (m_supplyTubes.tubeOne)
			return 2;
		break;
	case kTubeFour:
		if (m_supplyTubes.tubeThree)
			return 1;
		else if (m_supplyTubes.tubeTwo)
			return 2;
		else if (m_supplyTubes.tubeOne)
			return 3;
		break;
	default:
		return -1;
		break;
	}
}

/**
 * @brief Home the conveyor.
 * @details This is a blocking method
 * that drives the conveyor to its home
 * position and zeros it there.
 */
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

/**
 * @brief Close the gripper.
 * @details This closes the gripper
 * by calling the setGripper(true) 
 * method.
 */
void closeGripper()
{
	setGripper(true);
}

/**
 * @brief Open the gripper.
 * @details This opens the gripper
 * by calling the setGripper(false) 
 * method.
 */
void openGripper()
{
	setGripper(false);
}

/**
 * @brief Sets the gripper state.
 * @details This method sets the gripper 
 * state. True sets it to closed, and false
 * sets it to open.
 * 
 * @param closed Whether or not the gripper
 * should be closed.
 */
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
 * @brief Sets the fourbar state.
 * @details Sets the fourbar state. True
 * extends the fourbar, and false
 * retracts it.
 * 
 * @param out Whether or not the fourbar
 * mechanism should be extended.
 */
void setFourBar(bool out)
{
	if(out)
	{
		m_lineup.write(30);
	}
	else
	{
		m_lineup.write(2);
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
	changeState(kPreviousRobotState);
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
	noInterrupts();
	interruptCount++;
	if (_sendhb && (interruptCount % kHeartbeatPeriod) == 0)
	{
		sendHeartbeat();
	}
	if((has_old_rod || has_new_rod) && (interruptCount % kHeartbeatPeriod) == 0)
	{
		sendRadiation();
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

void sendRadiation()
{
	m_packetQueue.push(has_old_rod? kPktRadiationAlertLow:kPktRadiationAlertHigh);
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

/**
 * @brief Calibrate the QTRRC line sensor.
 * @details This method calibrates the line sensor.
 * It was taken directly with some modification from
 * the Pololu library. This method is blocking.
 */
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

/**
 * @brief Read the line position.
 * @details This method uses the line sensor
 * to take a reading of the position of the line.
 * It also updates the global line sensor accumulated
 * value.
 * 
 * @return The line position (from 0-7000).
 */
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

/**
 * @brief Drives the robot to a line.
 * @details Tracks a line up to another line
 * and then stops the robot. This method is
 * blocking.
 */
void trackToLine()
{
	unsigned int pos = readLinePosition();
	unsigned long strtTime = millis();
	while (millis() - strtTime < 300)
	{ 
		pos = readLinePosition();
		trackLine(pos);
	}
	while (sensorAccum < 4000)
	{
		pos = readLinePosition();
		trackLine(pos);
	}
	stopDrive();
}

/**
 * @brief Drives the robot to a line in reverse.
 * @details Tracks a line up to another line
 * and then stops the robot. This method tracks
 * in reverse. This method is blocking.
 */
void trackToLineReverse()
{
	unsigned int pos = readLinePosition();
	unsigned long strtTime = millis();
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

/**
 * @brief Track a line.
 * @details Drive the robot at a relatively constant
 * speed along a line. This method is non blocking.
 * 
 * @param int The position of the line.
 */
void trackLine(unsigned int position)
{
	int adjustedPosition = position - 3500 + 250;
	int leftDrive = (15 - (adjustedPosition * kLineTrackingP));
	int rightDrive = (15 + (adjustedPosition * kLineTrackingP));
	tankDrive(leftDrive, rightDrive);
}

/**
 * @brief Track a line in reverse.
 * @details Drive the robot in reverse at a relatively constant
 * speed along a line. This method is non blocking.
 * 
 * @param int The position of the line.
 */
void trackLineReverse(unsigned int position)
{
	int adjustedPosition = position - 3500 + 250;
	int leftDrive = (- 15 - (adjustedPosition * kLineTrackingP));
	int rightDrive = (- 15 + (adjustedPosition * kLineTrackingP));
	tankDrive(leftDrive, rightDrive);
}

/**
 * @brief Drives the robot.
 * @details Drives the robot using 
 * tank drive. The inputs are a value from
 * -90 to 90, unlike the servo object. This
 * change was made here to make driving the robot
 * a bit more intuative, with negative numbers
 * meaning backwards motion.
 * 
 * @param leftSpeed The speed of the left wheels.
 * @param rightSpeed The speed of the right wheels.
 */
void tankDrive(int leftSpeed, int rightSpeed)
{
	(rightSpeed > 0) ? rightSpeed += 10 : rightSpeed -= 10;
	leftSpeed = leftSpeed + 90;
	rightSpeed = rightSpeed + 90;
	(leftSpeed < 0) ? leftSpeed = 0 : (leftSpeed > 180) ? leftSpeed = 180 : leftSpeed = leftSpeed;
	(rightSpeed < 0) ? rightSpeed = 0 : (rightSpeed > 180) ? rightSpeed = 180 : rightSpeed = rightSpeed;
	m_frontRight.write(rightSpeed);
	m_frontLeft.write(180 - leftSpeed);
	m_rearLeft.write(180 - leftSpeed);
	m_rearRight.write(rightSpeed);
}

/**
 * @brief Stops drive.
 * @details Stops the robot drive motors.
 */
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
	unsigned long intTime = millis();
	switch (dir)
	{
	case kTurnRight:
		while (millis() - intTime < 500) { trackLine(readLinePosition()); }
		while (sensorValues[1] < 500) 
		{
			readLinePosition();
			tankDrive(15, -15); 
		}
		stopDrive();
		break;
	case kTurnLeft:
		while (millis() - intTime < 500) { trackLine(readLinePosition()); }
		while (sensorValues[6] < 500) 
		{ 
			readLinePosition();
			tankDrive(-25, 15); 
		}
		stopDrive();
		break;
	}
}

/**
 * @brief Turns the robot 180 degrees.
 * @details Turns the robot around, stopping
 * after passing a specfied number of lines. 
 * It also turns in the specified direction.
 * 
 * @param dir The direction to turn.
 * @param expectedLines The number of lines
 * that are expected to be passed.
 */
void turnAround(int dir, int expectedLines)
{
	unsigned long intTime = millis();
	switch (dir)
	{
	case kTurnRight:
		while(millis() - intTime < 200) { tankDrive(-20, -20); }
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
		while(millis() - intTime < 200) { tankDrive(-20, -20); }
		stopDrive();
		for (int i = 0; i < expectedLines; i++)
		{
			while (sensorValues[6] < 500)
			{ 
				readLinePosition();
				tankDrive(-20, 20); 
			}
			while (sensorValues[6] > 450)
			{ 
				readLinePosition();
				tankDrive(-20, 20); 
			}
		}
		stopDrive();
		break;
	}
}
