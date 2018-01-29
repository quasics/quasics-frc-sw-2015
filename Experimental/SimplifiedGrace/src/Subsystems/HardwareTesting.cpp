/*
 * HardwareTesting.cpp
 *
 *  Created on: Dec 14, 2017
 *      Author: healym
 */

#include "HardwareTesting.h"
#include "../RobotMap.h"

HardwareTesting::HardwareTesting() : Subsystem("HardwareTesting") {
	spareMotor1 = RobotMap::spareMotor1;
	spareMotor2 = RobotMap::spareMotor2;
	spareMotor2->SetInverted(true);		// Needs to run in the opposite direction as motor 1
}

void HardwareTesting::setSpareMotor1Power(double power) {
	spareMotor1->Set(power);
}
void HardwareTesting::setSpareMotor2Power(double power) {
	spareMotor2->Set(power);
}
