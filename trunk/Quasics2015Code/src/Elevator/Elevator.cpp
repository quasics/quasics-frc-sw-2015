/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */
#include "Elevator.h"

	Elevator::Elevator(int leftMotorPort, int rightMotorPort):
	leftMotor (leftMotorPort),
	rightMotor (rightMotorPort)
	{

	}

	void Elevator::Up (float DurationSeconds){
		leftMotor.Set(Relay::kForward);
		rightMotor.Set(Relay::kForward);
		Wait(DurationSeconds*1000);
		leftMotor.Set(Relay::kOff);
		rightMotor.Set(Relay::kOff);
	}
	void Elevator::Down (float DurationSeconds){
			leftMotor.Set(Relay::kReverse);
			rightMotor.Set(Relay::kReverse);
			Wait(DurationSeconds*1000);
			leftMotor.Set(Relay::kOff);
			rightMotor.Set(Relay::kOff);
	}
	void Elevator::Off (float DurationSeconds){
			leftMotor.Set(Relay::kOff);
			rightMotor.Set(Relay::kOff);
			Wait(DurationSeconds*1000);
			leftMotor.Set(Relay::kOff);
			rightMotor.Set(Relay::kOff);
	}
