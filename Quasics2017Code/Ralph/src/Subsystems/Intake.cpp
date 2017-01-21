/*
 * Intake.cpp
 *
 *  Created on: Jan 20, 2017
 *      Author: axf105
 */

#include "Intake.h"
#include "../RobotMap.h"
#include "WPILib.h"

Intake::Intake() : Subsystem("Intake") {
	intakeMotor = RobotMap::intakeMotor;

}

Intake::~Intake() {
	// TODO Auto-generated destructor stub

}


void Intake::TurnOn(double power) {
	intakeMotor->Set(power);
}
void Intake::TurnOff() {
	intakeMotor->StopMotor();
}
