/*
 * Elevator.h
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */

#ifndef SRC_ELEVATOR_ELEVATOR_H_
#define SRC_ELEVATOR_ELEVATOR_H_

#include "WPILib.h"

class Elevator{
public:
	//Constructor
	Elevator(int liftMotorPort);

	//Commands
	void Up ();
	void Down ();
	void Off ();

	void AutoUpInit (float durationSeconds);
	void AutoDownInit (float durationSeconds);
	void AutoProcess ();

	bool GoingUp();
	bool GoingDown ();


private:
	Timer autoTimer;
	Victor liftMotor;

	enum Status {
		kGoingUp, kGoingDown, kOff
	};

	Status activeStatus;
	float targetDuration;



};

#endif /* SRC_ELEVATOR_ELEVATOR_H_ */
