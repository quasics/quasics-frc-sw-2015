#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_
#include "WPILib.h"

class ProtoMan: public IterativeRobot {
public:
	ProtoMan();

private:
	Joystick stick;
	Victor left;
	Victor right;
	Encoder leftEncoder;
	Encoder rightEncoder;

	enum LiftState{
		kLifting, kDropping, kOff
	};
	LiftState liftState;
	LiftState liftStatePrevious;

	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestInit();
	void TestPeriodic ();

};

#endif /* SRC_ROBOT_H_ */
