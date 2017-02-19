/*
 * Gear.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: axf105
 */

#include "Gear.h"
#include "../RobotMap.h"
#include "WPILib.h"

Gear::Gear() :
		Subsystem("Gear") {
	gearServo = RobotMap::gearServo;
	doorOpen = false;

}

Gear::~Gear() {
	gearServo = 0;
}

void Gear::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen) {
		gearServo->Set(openValue);
		SmartDashboard::PutBoolean("Gear Door Open", true);
	} else {
		gearServo->Set(closeValue);
		SmartDashboard::PutBoolean("Gear Door Open", false);
	}
}

bool Gear::Get() {
	return doorOpen;
}

double Gear::GetPosition() {
	return gearServo->GetPosition();
}
