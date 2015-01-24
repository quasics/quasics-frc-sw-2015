/*
 * Robot.h
 *
 *  Created on: Jan 20, 2015
 *      Author: raymond healy
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_
#include "Includes.h"



class Robot: public IterativeRobot {
public:
	Robot ();

private:
	DriveTrain driveBase;
	Gamepad powerPad;

	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();

};



#endif /* SRC_ROBOT_H_ */


