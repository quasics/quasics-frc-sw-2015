/*
 * Gear.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: axf105
 */

#include "Gear.h"
#include "../RobotMap.h"

// CODE_REVIEW(mjh): Would it make sense to break this into two separate
// subsystems (one for the door, one for the kicker), and then use them
// together in a CommandGroup (running in parallel)?  This seems like it
// might yield a cleaner breakdown.
Gear::Gear() : Subsystem("Gear") {
	gearServo = RobotMap::gearServo;
	gearServoKicker = RobotMap::gearServoKicker;
	doorOpen = false;
	kickerExtended = false;
}

Gear::~Gear() {
	gearServo = nullptr;
	gearServoKicker = nullptr;
}

void Gear::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen)
		gearServo->Set(openValue);
	else
		gearServo->Set(closeValue);
}


void Gear::SetKicker(bool isExtended){
	kickerExtended = isExtended;
	if (isExtended)
		gearServoKicker->Set(kickerOut);
	else
		gearServoKicker->Set(kickerIn);
}



bool Gear::Get() {
	return doorOpen;
}


bool Gear::GetKicker(){
	return kickerExtended;

}

double Gear::GetPosition (){
	return gearServo->GetPosition();
	return gearServoKicker->GetPosition();
}
