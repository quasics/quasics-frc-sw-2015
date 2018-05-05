#include "SetLightDynamics.h"

SetLightDynamics::SetLightDynamics(ArduinoController::BrightnessMode dynamic) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::arduinoController.get());
	kBrightnessMode = dynamic;
}

// Called just before this Command runs the first time
void SetLightDynamics::Initialize() {
	Robot::arduinoController->SetLightDynamic(kBrightnessMode);
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
