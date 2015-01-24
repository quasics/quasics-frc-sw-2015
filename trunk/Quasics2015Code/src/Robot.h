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
	#define SlowMultiplier .5
	#define NormalMultiplier .75
	#define TurboMultiplier 1
	#define DeadbandWidth 0.05
	#define inPerTick 0.0524

	Talon leftFront;
	Talon leftRear;
	Talon rightFront;
	Talon rightRear;

	Joystick powerPad;


	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();

};



#endif /* SRC_ROBOT_H_ */


