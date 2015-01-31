/*
 * Grabber.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */
#include "Grabber.h"

	Grabber::Grabber (int screwMotorPort):
	screwMotor (screwMotorPort),
	moving (false),
	targetDuration (0),
	currentStatus (kNarrow)
	{

	}
	void Grabber::ChangeSizeStart (Status TargetSize){
		switch (currentStatus){
		case (kNarrow):
				switch (TargetSize){
				case (kCan):
						targetDuration = NarrowToCanTime;
						timer.Start();
						screwMotor.Set(Relay::kForward);
					break;
				case (kWide):
						targetDuration = NarrowToWideTime;
						timer.Start();
						screwMotor.Set(Relay::kForward);
					break;
				default:
					break;
				}
			break;
		case (kCan):
				switch (TargetSize){
				case (kNarrow):
						targetDuration = NarrowToCanTime;
						timer.Start();
						screwMotor.Set(Relay::kReverse);
					break;
				case (kWide):
						targetDuration = CanToWideTime;
						timer.Start();
						screwMotor.Set(Relay::kForward);
					break;
				default:
					break;
				}
			break;
		case (kWide):
				switch (TargetSize){
				case (kNarrow):
						targetDuration = NarrowToWideTime;
						timer.Start();
						screwMotor.Set(Relay::kReverse);
					break;
				case (kCan):
						targetDuration = CanToWideTime;
						timer.Start();
						screwMotor.Set(Relay::kReverse);
					break;
				default:
					break;
				}
			break;
		default:
			break;
		}
	}

	void Grabber::ChangeSizeProcess (){
		if (timer.Get() >= targetDuration){
			screwMotor.Set(Relay::kOff);
			timer.Stop();
			timer.Reset();
			targetDuration = 0;
		}
	}


	bool Grabber::IsMoving(){
		return (moving);
	}
	bool Grabber::Wide (){
		if (currentStatus == kWide){
			return (true);
		}
		else {
			return (false);
		}
	}

	bool Grabber::Narrow (){
		if (currentStatus == kNarrow){
			return (true);
		}
		else {
			return (false);
		}
	}

	bool Grabber::Can (){
		if (currentStatus == kCan){
			return (true);
		}
		else{
			return (false);
		}
	}
