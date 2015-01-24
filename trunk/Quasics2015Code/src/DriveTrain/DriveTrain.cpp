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
		TargetDegrees(0), TargetDistanceIn(0),

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

}
void DriveTrain::AutoTurnStart(float degrees) {

}
void DriveTrain::AutoProcess() {

}

//Sensors
void DriveTrain::ResetSensor(driveSensor whichSensor) {

}
float DriveTrain::GetSensorValue(driveSensor whichSensor) {
	return 0;

}
float DriveTrain::GetSpeed(driveSide whichSide, speedUnit whichSpeed) {
	return 0;
}

//Misc
float DriveTrain::GetDrivePowerLevel(driveSide oneSide) {
	return 0;
}
bool DriveTrain::AutoTurning() {
	return false;
}
bool DriveTrain::AutoDriving() {
	return false;
}
void DriveTrain::EndDriveAuto() {

}
