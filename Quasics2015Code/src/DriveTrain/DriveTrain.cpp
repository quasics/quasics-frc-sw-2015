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
		lastLeftPowerValue(0), lastRightPowerValue(0), AutoStatus(Disabled), TargetDegrees(
				0), TargetDistanceIn(0), leftTrimMult(1), rightTrimMult(1),

		leftFront(fLPort), leftRear(rLPort), rightFront(fRPort), rightRear(
				rRPort),

		/*leftTrim(lEncoderPortA), rightTrim(rEncoderPortA),*/leftTrim(
				lEncoderPortA, lEncoderPortB, false), rightTrim(rEncoderPortA,
				rEncoderPortB, true),

		gyro(gyroPort)

{

}

//Functions
//Teleop Power Setting
/*Tank Drive
 * Set the power to both sides manually
 */
void DriveTrain::SetDrivePower(float leftDrivePower, float rightDrivePower) {
	leftFront.Set(leftDrivePower);
	leftRear.Set(leftDrivePower);
	rightFront.Set(-rightDrivePower);
	rightRear.Set(-rightDrivePower);
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
	leftTrim.Reset();
	rightTrim.Reset();
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
		if (leftTrim.Get() * InPerTick >= TargetDistanceIn) {
			leftPower = 0;
		} else if (leftTrim.Get() * InPerTick > TargetDistanceIn - .5
				&& leftTrim.Get() * InPerTick < TargetDistanceIn) {
			leftPower = .25;
		} else {
			leftPower = .75;
		}

		if (rightTrim.Get() * InPerTick >= TargetDistanceIn) {
			rightPower = 0;
		} else if (leftTrim.Get() * InPerTick > TargetDistanceIn - .5
				&& leftTrim.Get() * InPerTick < TargetDistanceIn) {
			rightPower = .25;
		} else {
			rightPower = .75;
		}

		if (rightTrim.Get() * InPerTick >= TargetDistanceIn
				&& leftTrim.Get() * InPerTick >= TargetDistanceIn) {
			TargetDistanceIn = 0;
			leftTrim.Reset();
			rightTrim.Reset();
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
		leftTrim.Reset();
		break;
	case RightEncoder:
		rightTrim.Reset();
		break;
	case Gyroscope:
		gyro.Reset();
		break;
	}
}
float DriveTrain::GetSensorValue(driveSensor whichSensor) {
	switch (whichSensor) {
	case LeftEncoder:
		return leftTrim.Get();
	case RightEncoder:
		return rightTrim.Get();
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
	return (leftTrim.Get() * InPerTick);
}
float DriveTrain::GetRightDistanceIn() {
	return (rightTrim.Get() * InPerTick);
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

void DriveTrain::SmoothStick(float leftIn, float rightIn, float& leftOut,
		float& rightOut) {
	int leftConverted = int(leftIn * 10 + .5);
	int rightConverted = int(rightIn * 10 + .5);

	switch (leftConverted) {
	case (-10):
		leftOut = leftIn * 1;
		break;
	case (-9):
		leftOut = leftIn * .99;
		break;
	case (-8):
		leftOut = leftIn * .97;
		break;
	case (-7):
		leftOut = leftIn * .96;
		break;
	case (-6):
		leftOut = leftIn * .94;
		break;
	case (-5):
		leftOut = leftIn * .93;
		break;
	case (-4):
		leftOut = leftIn * .89;
		break;
	case (-3):
		leftOut = leftIn * .83;
		break;
	case (-2):
		leftOut = leftIn * .7;
		break;
	case (-1):
		leftOut =0;
		break;
	case (0):
		leftOut =0;
		break;
	case (1):
		leftOut =0;
		break;
	case (2):
		leftOut = leftIn * 1;
		break;
	case (3):
		leftOut = leftIn * 1;
		break;
	case (4):
		leftOut = leftIn * 1;
		break;
	case (5):
		leftOut = leftIn * 1;
		break;
	case (6):
		leftOut = leftIn * 1;
		break;
	case (7):
		leftOut = leftIn * 1;
		break;
	case (8):
		leftOut = leftIn * 1;
		break;
	case (9):
		leftOut = leftIn * 1;
		break;
	case (10):
		leftOut = leftIn * 1;
		break;
	}
	switch (rightConverted) {
	case (-10):
		rightOut = rightIn * .97;
		break;
	case (-9):
		rightOut = rightIn * 1;
		break;
	case (-8):
		rightOut = rightIn * 1;
		break;
	case (-7):
		rightOut = rightIn * 1;
		break;
	case (-6):
		rightOut = rightIn * 1;
		break;
	case (-5):
		rightOut = rightIn * 1;
		break;
	case (-4):
		rightOut = rightIn * 1;
		break;
	case (-3):
		rightOut = rightIn * 1;
		break;
	case (-2):
		rightOut = rightIn * 1;
		break;
	case (-1):
		rightOut =0;
		break;
	case (0):
		rightOut =0;
		break;
	case (1):
		rightOut =0;
		break;
	case (2):
		rightOut = rightIn * .96;
		break;
	case (3):
		rightOut = rightIn * .93;
		break;
	case (4):
		rightOut = rightIn * .94;
		break;
	case (5):
		rightOut = rightIn * .93;
		break;
	case (6):
		rightOut = rightIn * .93;
		break;
	case (7):
		rightOut = rightIn * .92;
		break;
	case (8):
		rightOut = rightIn * .91;
		break;
	case (9):
		rightOut = rightIn * .92;
		break;
	case (10):
		rightOut = rightIn * .96;
		break;
	}
}
void DriveTrain::TrimTest(float power){
	leftTrim.Reset();
	rightTrim.Reset();
	SetDrivePower (power, power);
	Wait (1);
	printf(" %f \n Left Encoder: %d \n Right Encoder: %d \n",power ,leftTrim.Get(), rightTrim.Get());
	if (leftTrim.Get() == 0 || rightTrim.Get() == 0){
		printf (" Left Multiplier: 0 \n Right Multiplier: 0 \n \n");
	}
	else if (leftTrim.Get() / rightTrim.Get() > rightTrim.Get() / leftTrim.Get()){
		printf (" Left Multiplier: %f \n Right Multiplier: 1 \n \n", float(rightTrim.Get() / leftTrim.Get()));
	}
	else if (leftTrim.Get() / rightTrim.Get() < rightTrim.Get() / leftTrim.Get()){
		printf(" Left Multiplier: 1 \n Right Multiplier: %f \n \n", float(leftTrim.Get() / rightTrim.Get()));
	}
	else if (leftTrim.Get() / rightTrim.Get() == 1 && rightTrim.Get() / leftTrim.Get() == 1){
		printf(" Left Multiplier: 1 \n Right Multiplier: 1 \n \n");
	}
}
