#include "OutputTele.h"

#include "../Robot.h"
#include "../RobotVariables.h"

OutputTele::OutputTele(double power)
	: powerPercent(power), motorOn(false), buttonDown(false)
{
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::outtake.get());
}

// Called just before this Command runs the first time
void OutputTele::Initialize() {
	motorOn = false;
}

// Called repeatedly when this Command is scheduled to run
void OutputTele::Execute() {
	buttonDown = Robot::oi->getDriveStick()->GetRawAxis(AuxRightXAxis);
	if (buttonDown && motorOn){
		Robot::outtake->TurnOff();
		motorOn = false;
	}
	else if(!buttonDown && !motorOn){
		Robot::outtake->TurnOn(powerPercent);
		motorOn = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool OutputTele::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void OutputTele::End() {
	Robot::outtake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OutputTele::Interrupted() {
	Robot::outtake->TurnOff();
}
