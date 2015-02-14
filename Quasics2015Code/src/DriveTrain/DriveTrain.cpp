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

		leftDist(lEncoderPortA), rightDist(rEncoderPortA), leftTrim(
				lEncoderPortA, lEncoderPortB, false), rightTrim(rEncoderPortA, rEncoderPortB, true),

		gyro(gyroPort)

{

}

//Functions
//Teleop Power Setting
/*Tank Drive
 * Set the power to both sides manually
 */
//#define Dev_Mode_Off
void DriveTrain::SetDrivePower(float leftDrivePower, float rightDrivePower) {
#ifdef Dev_Mode_Off
	float leftPower;
	float rightPower;

	if (AutoStatus == Disabled && trimTimer.Get() >= .1) {
		SetTrim();
		trimTimer.Reset();
	}

	if (fabs(leftDrivePower - rightDrivePower) <= 0.125) {
		leftFront.Set(((leftDrivePower + rightDrivePower) / 2) * leftTrimMult);
		leftRear.Set(((leftDrivePower + rightDrivePower) / 2) * leftTrimMult);
		rightFront.Set(
				((leftDrivePower + rightDrivePower) / 2) * rightTrimMult);
		rightRear.Set(((leftDrivePower + rightDrivePower) / 2) * rightTrimMult);

		leftPower = ((leftDrivePower + rightDrivePower) / 2) * leftTrimMult;
		rightPower = ((leftDrivePower + rightDrivePower) / 2) * rightTrimMult;

	} else {
		leftFront.Set(leftDrivePower);
		leftRear.Set(leftDrivePower);
		rightFront.Set(rightDrivePower);
		rightRear.Set(rightDrivePower);

		leftPower = leftDrivePower;
		rightPower = rightDrivePower;
	}

	lastLeftPowerValue = leftPower;
	lastRightPowerValue = rightPower;
	leftTrim.Reset();
	rightTrim.Reset();
#else
	leftFront.Set(leftDrivePower);
	leftRear.Set(leftDrivePower);
	rightFront.Set(-rightDrivePower);
	rightRear.Set(-rightDrivePower);
#endif
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

const float kMinPowerThreshold = 0.001f;

 #define USE_BROKEN_TRIM_CODE

void DriveTrain::SetTrim() {
	const int32_t leftTrimReading = leftTrim.Get();
	const int32_t rightTrimReading = rightTrim.Get();
//	const bool leftMovingForward = leftTrim.GetDirection();
//	const bool rightMovingForward = rightTrim.GetDirection();

//	const int32_t leftVelocity = leftTrimReading * (leftMovingForward ? +1 : -1);
//	const int32_t rightVelocity = rightTrimReading * (rightMovingForward ? +1 : -1);

#ifdef USE_BROKEN_TRIM_CODE
	if (leftTrimReading > rightTrimReading && leftTrimReading != 0) {
		leftTrimMult = rightTrimReading / leftTrimReading;
		rightTrimMult = 1;
	} else if (rightTrimReading > leftTrimReading && rightTrimReading != 0) {
		leftTrimMult = 1;
		rightTrimMult = leftTrimReading / rightTrimReading;
	} else {
		leftTrimMult = 1;
		rightTrimMult = 1;
	}
#else
	const bool rightStickZeroed = fabs(lastRightPowerValue)
			> kMinPowerThreshold;
	const bool leftStickZeroed = fabs(lastLeftPowerValue) > kMinPowerThreshold;

	if (leftStickZeroed && rightStickZeroed) {
		// No power: both joysticks at rest
		leftTrimMult = 0;
		rightTrimMult = 0;
	} else if (leftStickZeroed) {
		// Power on right, left is at rest position
		leftTrimMult = 0;
		rightTrimMult = 1;
	} else if (rightStickZeroed) {
		// Power on left, right is at rest position
		leftTrimMult = 1;
		rightTrimMult = 0;
	} else {
		// Power on both left and right: need to figure out
		// correct ratio to allow encoders to be adjusted

		// If both wheels aren't heading in the desired directions, then
		// adjusting trim here (based on current encoder readings) probably
		// isn't going to help any.  For example, if the stick says "left forward", and
		// the wheel has been moving backward, then modifying based on
		// old data doesn't make sense.
		const bool wantLeftMovingForward = (lastLeftPowerValue > 0);
		const bool wantRightMovingForward = (lastRightPowerValue > 0);

		const bool leftWheelDirIsOK = (wantLeftMovingForward == leftMovingForward);
		const bool rightWheelDirIsOK = (wantRightMovingForward == rightMovingForward);

		if (leftWheelDirIsOK && rightWheelDirIsOK) {
			// OK, perform/apply trim calc.

		}
		else {
			// Don't bother with trim: we're changing directions now, and
			// will deal with trim next time.  Just use the raw values
			// for now.
			leftTrimMult = rightTrimMult = 1;
		}

	}
#endif
}
