#include "Outtake.h"

Outtake::Outtake() : Subsystem("Outtake") {
	outputMotor = RobotMap::outtakeMotor;
}

void Outtake::TurnOn(double power) {
	outputMotor->Set(power);
}
void Outtake::TurnOff() {
	outputMotor->StopMotor();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
