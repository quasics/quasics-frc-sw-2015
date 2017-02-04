#include "Climber.h"
#include "../RobotMap.h"

Climber::Climber() :
	Subsystem("Climber") {
	climberMotor = RobotMap::climberMotor;
}

Climber::~Climber() {
	climberMotor = 0;
}

void Climber::TurnOn(double power) {
	climberMotor->Set(power);
}

void Climber::TurnOff() {
	climberMotor->StopMotor();
}
