/*
 * Robot.h
 *
 *  Created on: Jun 22, 2017
 *      Author: Yogna
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();
};

#endif /* SRC_ROBOT_H_ */
