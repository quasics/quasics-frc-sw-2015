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
	GLaDOS();						//Constructor

private:
	DriveTrain driveBase;			//Drive Base
	Gamepad powerPad;				//Elevator Control
	Gamepad logicPad;				//Drive Control
	std::string CameraHost;			//Camera Location
	AxisCamera camera;				//Camera
	Elevator elevator;				//Elevator


	bool fpsDriveOn;				//Is FPS drive being used
	bool DriveSwitchButtonPrevious;	//Last value of the drive mode switch
	const float slowMult = .25;		//Slow Multiplier
	const float normalMult = .5;	//Normal Multiplier
	const float fastMult = .75;		//Fast Multiplier


	void RobotInit();				//Initializer
	void AutonomousInit();			//Auto Initializer
	void AutonomousPeriodic();		//Auto Period
	void TeleopInit();				//Teleop Initializer
	void TeleopPeriodic();			//Teleop Period
	void TestPeriodic();			//Test mode

};

#endif /* SRC_ROBOT_H_ */

