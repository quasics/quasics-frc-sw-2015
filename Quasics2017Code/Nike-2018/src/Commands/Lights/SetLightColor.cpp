#include "SetLightColor.h"

SetLightColor::SetLightColor(ArduinoController::ColorMode colorMode) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::arduinoController.get());
	kColorMode = colorMode;
}

// Called just before this Command runs the first time
void SetLightColor::Initialize() {
	Robot::arduinoController->SetLightColor(kColorMode);
}

// Called repeatedly when this Command is scheduled to run
void SetLightColor::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool SetLightColor::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void SetLightColor::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetLightColor::Interrupted() {

}
