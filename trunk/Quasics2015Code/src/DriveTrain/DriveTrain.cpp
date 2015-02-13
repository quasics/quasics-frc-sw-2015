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
		AutoStatus(Disabled), TargetDegrees(0), TargetDistanceIn(0), leftTrimMult(
				1), rightTrimMult(1),

		leftFront(fLPort), leftRear(rLPort), rightFront(fRPort), rightRear(
				rRPort),

		leftDist(lEncoderPortA), rightDist(rEncoderPortA), leftTrim(
				lEncoderPortB), rightTrim(rEncoderPortB),

		gyro(gyroPort)

{

}

//Functions
//Teleop Power Setting
/*Tank Drive
 * Set the power to both sides manually
 */
void DriveTrain::SetDrivePower(float leftDrivePower, float rightDrivePower) {
	if (AutoStatus == Disabled && trimTimer.Get() >= 1) {
		SetTrim ();
		trimTimer.Reset();
}
	if (fabs(leftDrivePower - rightDrivePower) <= 0.125) {
		leftFront.Set(((leftDrivePower+rightDrivePower)/2) * leftTrimMult);
		leftRear.Set(((leftDrivePower+rightDrivePower)/2) * leftTrimMult);
		rightFront.Set(((leftDrivePower+rightDrivePower)/2) * rightTrimMult);
		rightRear.Set(((leftDrivePower+rightDrivePower)/2) * rightTrimMult);
	}
	else{
		leftFront.Set(leftDrivePower);
		leftRear.Set(leftDrivePower);
		rightFront.Set(rightDrivePower);
		rightRear.Set(rightDrivePower);
	}
}
/*FPS Drive
 * Use 2 Axes, One for Throttle and one for Yaw Control
 *
 * Yaw modifier modeled off of the equation Y = -|2X| + 1, converted, in this case to
 * left_scale = 2(rudder_axis) + upper_bound and right_scale = 2(rudder_axis) + upper_bound
 * where upper_bound = 1
 */
void DriveTrain::FPSDrive(float throttlePower, float sideScale) {
	float leftScale = 1;
	float rightScale = 1;
	if (sideScale >= 0) {
		rightScale = -2 * sideScale + 1;
	} else {
		leftScale = 2 * sideScale + 1;
	}
	SetDrivePower(throttlePower * leftScale, throttlePower * rightScale);
}

//Auto mode Power Setting
void DriveTrain::AutoDriveStart(float distanceIn) {
	leftDist.Reset();
	rightDist.Reset();
	TargetDistanceIn = distanceIn;
	AutoStatus = Driving;
}
void DriveTrain::AutoTurnStart(float degrees) {
	gyro.Reset();
	TargetDegrees = degrees;
	AutoStatus = Turning;
}
void DriveTrain::AutoProcess() {
	float leftPower = 0;
	float rightPower = 0;
	switch (AutoStatus) {
	case Driving:
		if (leftDist.Get() * InPerTick >= TargetDistanceIn) {
			leftPower = 0;
		} else if (leftDist.Get() * InPerTick > TargetDistanceIn - .5
				&& leftDist.Get() * InPerTick < TargetDistanceIn) {
			leftPower = .25;
		} else {
			leftPower = .75;
		}

		if (rightDist.Get() * InPerTick >= TargetDistanceIn) {
			rightPower = 0;
		} else if (leftDist.Get() * InPerTick > TargetDistanceIn - .5
				&& leftDist.Get() * InPerTick < TargetDistanceIn) {
			rightPower = .25;
		} else {
			rightPower = .75;
		}

		if (rightDist.Get() * InPerTick >= TargetDistanceIn
				&& leftDist.Get() * InPerTick >= TargetDistanceIn) {
			TargetDistanceIn = 0;
			leftDist.Reset();
			rightDist.Reset();
			AutoStatus = Ready;
		}
		break;

	case Turning:
		if (TargetDegrees > 0 && TargetDegrees <= 180) {
			if (gyro.GetAngle() <= TargetDegrees - 2.5) {
				leftPower = .75;
				rightPower = -.75;
			} else if (gyro.GetAngle() > TargetDegrees - 2.5
					&& gyro.GetAngle() < TargetDegrees) {
				leftPower = .25;
				rightPower = -.25;
			} else {
				leftPower = 0;
				rightPower = 0;
				AutoStatus = Ready;
				gyro.Reset();
				TargetDegrees = 0;
			}
		} else {
			if (gyro.GetAngle() >= TargetDegrees + 2.5) {
				leftPower = -.75;
				rightPower = .75;
			} else if (gyro.GetAngle() < TargetDegrees + 2.5
					&& gyro.GetAngle() > TargetDegrees) {
				leftPower = -.25;
				rightPower = .25;
			} else {
				leftPower = 0;
				rightPower = 0;
				AutoStatus = Ready;
				gyro.Reset();
				TargetDegrees = 0;
			}
		}
		break;
	default:
		break;

	}
	SetDrivePower(leftPower, rightPower);
}

//Sensors
void DriveTrain::ResetSensor(driveSensor whichSensor) {
	switch (whichSensor) {
	case LeftEncoder:
		leftDist.Reset();
		break;
	case RightEncoder:
		rightDist.Reset();
		break;
	case Gyroscope:
		gyro.Reset();
		break;
	}
}
float DriveTrain::GetSensorValue(driveSensor whichSensor) {
	switch (whichSensor) {
	case LeftEncoder:
		return leftDist.Get();
	case RightEncoder:
		return rightDist.Get();
	case Gyroscope:
		return gyro.GetAngle();
	default:
		return 0;
	}
}
float DriveTrain::GetSpeed(driveSide whichSide, speedUnit whichSpeed) {
	return 0; //leave for end
}
float DriveTrain::GetLeftDistanceIn() {
	return (leftDist.Get() * InPerTick);
}
float DriveTrain::GetRightDistanceIn() {
	return (rightDist.Get() * InPerTick);
}

//Misc
bool DriveTrain::AutoTurning() {
	if (AutoStatus == Turning) {
		return true;
	} else {
		return false;
	}
}
bool DriveTrain::AutoDriving() {
	if (AutoStatus == Driving) {
		return true;
	} else {
		return false;
	}
}
void DriveTrain::EndDriveAuto() {
	AutoStatus = Disabled;
	trimTimer.Start();
}
void DriveTrain::SetTrim() {
	if (leftTrim.Get() > rightTrim.Get() && leftTrim.Get() == !0) {
		leftTrimMult = rightTrim.Get() / leftTrim.Get();
		rightTrimMult = 1;
	}
	else if (rightTrim.Get() > leftTrim.Get() && rightTrim.Get() == !0){
		leftTrimMult = 1;
		rightTrimMult = leftTrim.Get() / rightTrim.Get();
	}
	else {
		leftTrimMult = 1;
		rightTrimMult = 1;
	}
}
