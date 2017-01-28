/*
 * Gear.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: axf105
 */

#include "Gear.h"

Gear::Gear() : Subsystem("Gear") {
	// TODO Auto-generated constructor stub
	gearServo = RobotMap::gearServo;
	doorOpen = false;

}

Gear::~Gear() {
	// TODO Auto-generated destructor stub
}

void Gear::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen)
		gearServo->Set(openValue);
	else
		gearServo->Set(closeValue);
}

bool Gear::Get() {
	return doorOpen;
}
