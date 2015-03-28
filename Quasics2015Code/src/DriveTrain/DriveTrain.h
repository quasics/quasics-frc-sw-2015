/*
 * DriveTrain.h
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */

#ifndef SRC_DRIVETRAIN_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_DRIVETRAIN_H_

#include "WPILib.h"

class DriveTrain {
public:
	//Constructor
	DriveTrain(int fLPort, int fRPort, int rLPort, int rRPort,
			int lEncoderPortA, int lEncoderPortB, int rEncoderPortA,
			int rEncoderPortB, int gyroPort);

	//Enumeration
	enum driveSensor {
		LeftEncoder, RightEncoder, Gyroscope
	};
	enum driveSide {
		Left, Right, Both
	};
	enum speedUnit {
		FeetPerSec, MetersPerSec, MilesPerHr, kMPerHr
	};
	enum autoStatus {
		Ready, Driving, Turning, Disabled
	};


	//Functions
	//Teleop Power Setting
	void FPSDrive (float throttlePower, float sideScale);
	void SetDrivePower(float leftDrivePower, float rightDrivePower);

	//Auto mode Power Setting
	void AutoDriveStart(float distanceIn);
	void AutoTurnStart(float degrees);
	void AutoProcess();

	//Sensors
	void ResetSensor(driveSensor whichSensor);
	float GetSensorValue(driveSensor whichSensor);
	float GetSpeed(driveSide whichSide, speedUnit whichSpeed);
	float GetLeftDistanceIn();
	float GetRightDistanceIn();

	//Misc
	bool AutoTurning();
	bool AutoDriving();
	void EndDriveAuto();
	void SmoothStick (float leftIn, float rightIn, float& leftOut, float& rightOut);
	void TrimTest (float power);
	void SpeedTest(float turboPower = .75f, float normalPower = .5f, float slowPower = .25f);

private:

	enum direction {
		TurningLeft, TurningRight, Forward
	};

	const float InPerTick = 0.0524;

  float lastLeftPowerValue;
  float lastRightPowerValue;
  
	autoStatus AutoStatus;
	float TargetDegrees;
	float TargetDistanceIn;
	float leftTrimMult;
	float rightTrimMult;

	//Timers
	Timer trimTimer;

	//Motor Controllers
	Talon leftFront;
	Talon leftRear;
	Talon rightFront;
	Talon rightRear;

	//Sensors
//	Counter leftDist;
//	Counter rightDist;
	Encoder leftTrim;
	Encoder rightTrim;
	Gyro gyro;
};

#endif /* SRC_DRIVETRAIN_DRIVETRAIN_H_ */
