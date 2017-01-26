/*
 * Climber.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: axf105
 */

#include "Climber.h"
#include "../RobotMap.h"
#include "WPILib.h"

Climber::Climber() :  Subsystem("Climber")  {
	climberMotor = RobotMap::climberMotor;

}

Climber::~Climber() {
	// TODO Auto-generated destructor stub

}


void Climber::TurnOn(double power) {
	climberMotor->Set(power);
}
void Climber::TurnOff() {
	climberMotor->StopMotor();
}
