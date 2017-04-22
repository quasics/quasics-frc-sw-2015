#include "SetLightDynamics.h"

SetLightDynamics::SetLightDynamics(Arduino::BrightnessMode mode) {
	Requires (Robot::arduino.get());
	kMode = mode;
}

// Called just before this Command runs the first time
void SetLightDynamics::Initialize() {
	Robot::arduino->SetBrightnessMode(kMode);
}

// Called repeatedly when this Command is scheduled to run
void SetLightDynamics::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool SetLightDynamics::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void SetLightDynamics::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetLightDynamics::Interrupted() {
}
