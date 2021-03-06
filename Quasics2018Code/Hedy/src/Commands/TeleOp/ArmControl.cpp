#include <Commands/TeleOp/ArmControl.h>
#include "../../Robot.h"

ArmControl::ArmControl() {
	Requires(Robot::cubeManipulation.get());
}

// Called just before this Command runs the first time
void ArmControl::Initialize() {
	Robot::cubeManipulation->SetShoulderPower(0);
}

// Called repeatedly when this Command is scheduled to run
void ArmControl::Execute() {

	double rightPower = Robot::oi->getWinchPower();

	// If it's far enough away from the mid-point of the joystick, then
	// we'll treat it as a real power setting.  Otherwise, it's in the
	// "dead zone", and we'll bring the power to 0.
	//
	// TODO: Define the dead zone.  Right now, there is none.
	//       (Example: "if (fabs(rightPower) > SOME_NUMBER) {....} else {....}")
	if (rightPower) {
		Robot::cubeManipulation->SetShoulderPower(-(rightPower));
	} else {
		Robot::cubeManipulation->SetShoulderPower(0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ArmControl::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ArmControl::End() {
	Robot::cubeManipulation->SetShoulderPower(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ArmControl::Interrupted() {
	End();
}
