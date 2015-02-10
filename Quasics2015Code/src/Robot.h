/*
 * Robot.h
 *
 *  Created on: Jan 20, 2015
 *      Author: raymond healy
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_
#include "Includes.h"

class GLaDOS: public IterativeRobot {
public:
	GLaDOS();

private:
	DriveTrain driveBase;
	PS4Controller powerPad;
	Gamepad logicPad;
	AxisCamera camera;
	Elevator elevator;


	bool PissPoorDriveOn;
	bool LogicSwitchButtonPrevious;
	bool PS4SwitchButtonPrevious;

	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();

};

#endif /* SRC_ROBOT_H_ */

