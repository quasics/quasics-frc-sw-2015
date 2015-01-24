/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */

#include "DriveTrain.h"

DriveTrain::DriveTrain(int fLPort, int fRPort, int rLPort, int rRPort,
		int lEncoderPortA, int lEncoderPortB, int rEncoderPortA,
		int rEncoderPortB, int gyroPort) :
		AutoStatus(Disabled),TargetDegrees(0), TargetDistanceIn(0),

		leftFront(fLPort), leftRear(rLPort), rightFront(fRPort), rightRear(
				rRPort),

		leftEncoder(lEncoderPortA, lEncoderPortB), rightEncoder(rEncoderPortA,
				rEncoderPortB),

		gyro(gyroPort)

{

}

//Functions
//Teleop Power Setting
void DriveTrain::SetDrivePower(float leftDrivePower, float rightDrivePower) {
	leftFront.Set(leftDrivePower);
	leftRear.Set(leftDrivePower);
	rightFront.Set(-rightDrivePower);
	rightRear.Set(-rightDrivePower);
}

//Auto mode Power Setting
void DriveTrain::AutoDriveStart(float distanceIn) {
	TargetDistanceIn = distanceIn;
	AutoStatus = Driving;
}
void DriveTrain::AutoTurnStart(float degrees) {
	TargetDegrees = degrees;
	AutoStatus = Turning;
}
void DriveTrain::AutoProcess() {
	//FILL IN LATER
}

//Sensors
void DriveTrain::ResetSensor(driveSensor whichSensor) {
	switch (whichSensor){
	case LeftEncoder:
		leftEncoder.Reset();
		break;
	case RightEncoder:
		rightEncoder.Reset();
		break;
	case Gyroscope:
		gyro.Reset();
		break;
	}
}
float DriveTrain::GetSensorValue(driveSensor whichSensor) {
	switch (whichSensor) {
	case LeftEncoder:
		return leftEncoder.Get();
	case RightEncoder:
		return rightEncoder.Get();
	case Gyroscope:
		return gyro.GetAngle();
	default:
		return 0;
	}
}
float DriveTrain::GetSpeed(driveSide whichSide, speedUnit whichSpeed) {
	return 0;//leave for end
}

//Misc

bool DriveTrain::AutoTurning() {
		if (AutoStatus == Turning){
			return true;
		}
		else{
			return false;
		}
}
bool DriveTrain::AutoDriving() {
	if (AutoStatus == Driving){
				return true;
			}
			else{
				return false;
			}
}
void DriveTrain::EndDriveAuto() {
	AutoStatus = Disabled;
}
