#include <Commands/MoveForward.h>
#include "../Robot.h"

// The number of times per second that Execute() is called while the command
// is running.
const uint32_t kCyclesPerSecond = 50;

MoveForward::MoveForward(double powerLevel, double seconds)
: power(powerLevel),
  cyclesToRun(uint32_t(kCyclesPerSecond * seconds) + .5),
  counter(0)
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void MoveForward::Initialize() {
	// Note: This is going to produce pretty jerky motion, especially at
	// anything near full power.  A better approach might be to start out at
	// low power, ramping up to speed, and ramping down as we get close to
	// the end of the allotted time.
	Robot::driveBase->SetLeftPower(power);
	Robot::driveBase->SetRightPower(-power);
	counter = 0;
}

// Called repeatedly when this Command is scheduled to run
void MoveForward::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForward::IsFinished() {
	return counter >= cyclesToRun;
}

// Called once after isFinished returns true
void MoveForward::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForward::Interrupted() {
	Robot::driveBase->Stop();
}
