#include "FuelExhaustTeleOp.h"

#include "../Robot.h"
#include "../RobotVariables.h"

FuelExhaustTeleOp::FuelExhaustTeleOp()
:  actuatorOpen(false), buttonDown(false), buttonPrevious(false)
{
	Requires(Robot::fuelExhaustGate.get());
}

// Called just before this Command runs the first time
void FuelExhaustTeleOp::Initialize() {
	actuatorOpen = false;
}

// Called repeatedly when this Command is scheduled to run
void FuelExhaustTeleOp::Execute() {
	buttonDown = Robot::oi->getDriveStick()->GetRawAxis(AuxRightYAxis);
	if (buttonDown != buttonPrevious){
		actuatorOpen = !actuatorOpen;
	}
	Robot::fuelExhaustGate->Set(actuatorOpen);
	buttonPrevious = buttonDown;

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
