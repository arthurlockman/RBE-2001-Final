#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "UltrasonicRangefinder.h"
#include "PPM.h"
#include "Arduino.h"
#include "Servo.h"
#include "LimitSwitch.h"
#include "LSM303.h"
#include "QTRSensors.h"
#include "LiquidCrystal.h"
#include "ReactorProtocol.h"
#include "BluetoothClient.h"
#include "BluetoothMaster.h"
#include "TimerThree.h"
#include "Encoder.h"
#include "Alarm.h"
#include "Wire.h"
#include "QTRSensors.h"
#include "QueueList.h"
#include "BumpSensor.h"

#define DEBUG 1
#define TEAM_NUMBER 2

///PID Values
static const float kLineTrackingP = 0.01;

///Operation States
enum RobotStates
{
	kStartup,
	kWaitForBluetooth,
	kTankDrive,
	kPaused,
};

enum Direction
{
	kNorth,
	kSouth,
	kEast,
	kWest
};

///The different motors that are attached to the robot.
enum Motors
{
	kFrontRightMotor = 6, ///< The front right motor.
	kFrontLeftMotor = 7, ///< The front left motor.
	kRearRightMotor = 8, ///< The rear right motor.
	kRearLeftMotor = 9, ///< The rear left motor.
	kClawMotor = 10,
	kConveyerMotor = 11,
	kFourBarMotor = 12
};

///The digital inputs on the robot.
enum DigitalInputs
{
	kEncoder1 = 0, //<Digital Pin 2
	kEncoder2 = 1, //<Digital Pin 3
	kLineSensor7 = 22,
	kLineSensor6 = 23,
	kLineSensor5 = 24,
	kLineSensor4 = 25,
	kLineSensor3 = 26,
	kLineSensor2 = 27,
	kLineSensor1 = 28,
	kLineSensor0 = 29,
	kLineSensorLED = 30,
	kStartButtonInput = 37
};

///The Analog inputs on the robot.
enum AnalogInputs
{
	kAccX = 0,
	kAccY = 1,
	kAccZ = 2
};

///The joystick axis on the robot.
enum JoystickAxis
{
	kRightStickX = 1, ///< The right joystick's X axis.
	kRightStickY = 2, ///< The right joystick's Y axis.
	kLeftStickY = 3, ///< The left joystick's Y axis.
	kLeftStickX = 4, ///< The left joystick's X axis.
	kBackLeftButtons = 5, ///< The joystick's back left buttons.
	kBackRightButtons = 6 ///< The joystick's back right buttons.
};

///The digital output pins on the robot.
enum DigitalOutPins
{
	kRadiationAlarm = 3, ///< The pin that has the radiation alarm on it.
	kCompassPower = 52
};

///Bluetooth Packet Types
enum BTPacketTypes
{
	kBTStorageTubeAvailable = 0x01,
	kBTSupplyTubeAvailable = 0x02,
	kBTRadiationAlert = 0x03,
	kBTStopMovement = 0x04,
	kBTResumeMovement = 0x05,
	kBTRobotStatus = 0x06,
	kBTHeartbeat = 0x07
};

///Enum to hold bluetooth addresses.
enum RobotAddresses
{
	kAddressFMS = 0x00,
	kAddressRobot = 0x02 
};

///Radiation Alert Messages
enum RadiationAlertMessages
{
	kRadiationCarryingSpent = 0x2C,
	kRadiationCarryingNew = 0xFF
};

///Movement status messages
enum MovementStatusMessages
{
	kMovementStopped = 0x01,
	kMovementTeleop = 0x02,
	kMovementAuto = 0x03
};

///Gripper status messages
enum GripperStatusMessages
{
	kGripperNoRod = 0x01,
	kGripperHasRod = 0x02
};

///Operation status messages
enum OperationStatusMessages
{
	kOperationGripEngage = 0x01,
	kOperationGripRelease = 0x02,
	kOperationDrivingToRod = 0x03,
	kOperationDrivingToStorage = 0x04,
	kOperationDrivingToSupply = 0x05,
	kOperationIdle = 0x06
};

///A structure to hold a bluetooth packet.
struct BTPacket
{
	byte* packetData;
	int packetSize;
};

///Availability information about the tubes.
struct TubeAvailability
{
	int tubeOne;
	int tubeTwo;
	int tubeThree;
	int tubeFour;
};

/**
 * @brief Tells whether or not a value is within a certain tolerance of another value.
 * @param value The value to check.
 * @param desiredValue The value to check value against.
 * @param tolerance The desired tolerance.
 * @return A boolean, whether or not the value is within the tolerance.
 */
template <typename T>
inline bool WithinTolerance(T value, T desiredValue, T tolerance)
{
	if ((value < desiredValue + tolerance) && (value > desiredValue - tolerance))
    	return true;
	else
    	return false;
};

/**
 * @brief Applies a deadband to a number.
 * @details Applies a deadband to a number. Useful
 * for joystick input and motor output.
 * 
 * @param value The value to apply the deadband to.
 * @param deadbandUpper The upper limit of the deadband.
 * @param deadbandLower The lower limit of the deadband.
 * @param deadbandMid The middle value of the deadband.
 * @return Returns the deadbanded value.
 */
template <typename T>
inline T Deadband(T value, T deadbandUpper, T deadbandLower, T deadbandMid)
{
	if (value <= deadbandUpper && value >= deadbandLower)
		return deadbandMid;
	else 
		return value;
};

#endif
