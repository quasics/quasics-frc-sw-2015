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
	Elevator(int leftMotorPort, int rightMotorPort);

	//Commands
private:
	Relay leftMotor;
	Relay rightMotor;

};

#endif /* SRC_ELEVATOR_ELEVATOR_H_ */
