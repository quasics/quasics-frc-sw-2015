#include "TurnLightOff.h"
#include "Robot.h"

TurnLightOff::TurnLightOff(): frc::Command() {
	Requires(Robot::lighting.get());
}

// Called just before this Command runs the first time
void TurnLightOff::Initialize() {
	Robot::lighting->WriteOff();
}

// Make this return true when this Command no longer needs to run execute()
bool TurnLightOff::IsFinished() {
    return true;
}
