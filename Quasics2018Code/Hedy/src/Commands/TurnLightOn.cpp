#include "TurnLightOn.h"
#include "Robot.h"

TurnLightOn::TurnLightOn(): frc::Command() {
	Requires(Robot::lighting.get());
}

// Called just before this Command runs the first time
void TurnLightOn::Initialize() {
	Robot::lighting->WriteOn();
}

// Make this return true when this Command no longer needs to run execute()
bool TurnLightOn::IsFinished() {
    return true;
}
