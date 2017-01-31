/*
 * Gear.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: axf105
 */

#include "Gear.h"

Gear::Gear() : Subsystem("Gear") {
	gearServo = RobotMap::gearServo;
	doorOpen = false;

}

Gear::~Gear() {
	gearServo = 0;
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
