/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */
#include "Elevator.h"

	Elevator::Elevator(int leftMotorPort, int rightMotorPort):
	leftMotor (leftMotorPort),
	rightMotor (rightMotorPort),
	activeStatus (kOff),
	targetDuration (0)
	{

	}

	void Elevator::Up (){
		leftMotor.Set(Relay::kForward);
		rightMotor.Set(Relay::kForward);
		activeStatus = kGoingUp;
	}
	void Elevator::Down (){
		leftMotor.Set(Relay::kReverse);
		rightMotor.Set(Relay::kReverse);
		activeStatus = kGoingDown;
	}
	void Elevator::Off (){
		leftMotor.Set(Relay::kOff);
		rightMotor.Set(Relay::kOff);
		activeStatus = kOff;
	}

	void Elevator::AutoUpInit (float durationSeconds){
		leftMotor.Set(Relay::kForward);
		rightMotor.Set(Relay::kForward);
		activeStatus = kGoingUp;
		targetDuration = durationSeconds;
		autoTimer.Start();
	}
	void Elevator::AutoDownInit (float durationSeconds){
		leftMotor.Set(Relay::kReverse);
		rightMotor.Set(Relay::kReverse);
		activeStatus = kGoingDown;
		targetDuration = durationSeconds;
		autoTimer.Start();
	}
	void Elevator::AutoProcess(){
		switch (activeStatus){
		case (kGoingUp):
			if (autoTimer.Get() >= targetDuration){
				leftMotor.Set(Relay::kOff);
				rightMotor.Set(Relay::kOff);
				activeStatus = kOff;
				autoTimer.Stop();
				autoTimer.Reset();
				targetDuration = 0;
			}
			break;
		case (kGoingDown):
			if (autoTimer.Get() >= targetDuration){
				leftMotor.Set(Relay::kOff);
				rightMotor.Set(Relay::kOff);
				activeStatus = kOff;
				autoTimer.Stop();
				autoTimer.Reset();
				targetDuration = 0;
				}
			break;
		default:
			break;
		}
	}


	bool Elevator::GoingUp(){
		if (activeStatus == kGoingUp){
			return (true);
		}
		else {
			return (false);
		}
	}
	bool Elevator::GoingDown (){
		if (activeStatus == kGoingDown){
			return (true);
		}
		else {
			return (false);
		}

	}
