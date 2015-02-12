/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */
#include "Elevator.h"

	Elevator::Elevator(int liftMotorPort):
	liftMotor (liftMotorPort),
	activeStatus (kOff),
	targetDuration (0)
	{

	}

	void Elevator::Up (){
		liftMotor.Set(1);
		activeStatus = kGoingUp;
	}
	void Elevator::Down (){
		liftMotor.Set(-1);
		activeStatus = kGoingDown;
	}
	void Elevator::Off (){
		liftMotor.Set(0);
		activeStatus = kOff;
	}

	void Elevator::AutoUpInit (float durationSeconds){
		liftMotor.Set(1);
		activeStatus = kGoingUp;
		targetDuration = durationSeconds;
		autoTimer.Start();
	}
	void Elevator::AutoDownInit (float durationSeconds){
		liftMotor.Set(-1);
		activeStatus = kGoingDown;
		targetDuration = durationSeconds;
		autoTimer.Start();
	}
	void Elevator::AutoProcess(){
		switch (activeStatus){
		case (kGoingUp):
			if (autoTimer.Get() >= targetDuration){
				liftMotor.Set(0);
				activeStatus = kOff;
				autoTimer.Stop();
				autoTimer.Reset();
				targetDuration = 0;
			}
			break;
		case (kGoingDown):
			if (autoTimer.Get() >= targetDuration){
				liftMotor.Set(0);
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
