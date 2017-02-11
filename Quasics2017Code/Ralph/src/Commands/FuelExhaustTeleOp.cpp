#include "FuelExhaustTeleOp.h"

#include "../Robot.h"
#include "../RobotVariables.h"

FuelExhaustTeleOp::FuelExhaustTeleOp()
:  actuatorOpen(false), buttonDown(false)
{
	Requires(Robot::fuelExhaustGate.get());
}

// Called just before this Command runs the first time
void FuelExhaustTeleOp::Initialize() {
	actuatorOpen = false;
}

// Called repeatedly when this Command is scheduled to run
void FuelExhaustTeleOp::Execute() {
	buttonDown = Robot::oi->getDriveStick()->GetRawButton(OutputButton);
	if (buttonDown && actuatorOpen){
		Robot::fuelExhaustGate->Set(false);
		actuatorOpen = false;
	}
	else if(!buttonDown && !actuatorOpen){
		Robot::fuelExhaustGate->Set(true);
		actuatorOpen = true;
	}

}

// Make this return true when this Command no longer needs to run execute()
bool FuelExhaustTeleOp::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FuelExhaustTeleOp::End() {
	Robot::fuelExhaustGate->Set(false);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FuelExhaustTeleOp::Interrupted() {
	Robot::fuelExhaustGate->Set(false);

}
