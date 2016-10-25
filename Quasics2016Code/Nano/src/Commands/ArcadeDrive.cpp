#include "ArcadeDrive.h"

ArcadeDrive::ArcadeDrive() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
}

// Called just before this Command runs the first time
void ArcadeDrive::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool ArcadeDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ArcadeDrive::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ArcadeDrive::Interrupted() {

}
