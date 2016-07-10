#include "StopShooter.h"

StopShooter::StopShooter() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	Requires(Robot::shooter.get());
}

// Called just before this Command runs the first time
void StopShooter::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void StopShooter::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool StopShooter::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void StopShooter::End() {
	Robot::shooter->StopPiston();
	Robot::shooter->StopWheels();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StopShooter::Interrupted() {
	Robot::shooter->StopPiston();
	Robot::shooter->StopWheels();
}
