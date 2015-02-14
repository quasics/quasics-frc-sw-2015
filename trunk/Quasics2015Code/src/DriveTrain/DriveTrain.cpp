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
