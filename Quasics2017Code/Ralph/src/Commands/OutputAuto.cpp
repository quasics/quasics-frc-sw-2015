#include "OutputAuto.h"

#include "../Robot.h"
#include "../Subsystems/Outtake.h"


OutputAuto::OutputAuto(double power) {
	Requires(Robot::outtake.get());
	powerPercent = power;
}

// Called just before this Command runs the first time
void OutputAuto::Initialize() {
	Robot::outtake->TurnOn(powerPercent);
}

// Make this return true when this Command no longer needs to run execute()
bool OutputAuto::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void OutputAuto::End() {
	Robot::outtake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OutputAuto::Interrupted() {
	Robot::outtake->TurnOff();
}
