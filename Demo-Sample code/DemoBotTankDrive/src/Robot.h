/*
 * Robot.h
 *
 *  Created on: Jun 22, 2017
 *      Author: Yogna
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include "ControllerVariables.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit();
	void TeleopPeriodic();
private:
	const uint32_t leftFrontPort = 0;
	const uint32_t leftRearPort = 1;
	const uint32_t rightFrontPort = 2;
	const uint32_t rightRearPort = 3;
	const uint32_t joystickPort = 0;

	const bool isLeftReversed = false;

	const unsigned char leftYAxis = 1;
	const unsigned char rightYAxis = 3;

	const double slowMultiplier = -0.35;
	const double middleMultiplier = -0.65;
	const double turboMultiplier = -0.9;

	std::unique_ptr<SpeedController> leftFront;
	std::unique_ptr<SpeedController> leftRear;
	std::unique_ptr<SpeedController> rightFront;
	std::unique_ptr<SpeedController> rightRear;

	std::unique_ptr<Joystick> pilotStick;
};

#endif /* SRC_ROBOT_H_ */
