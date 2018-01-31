#include "ArmMoveUp.h"
#include "../../ControllerDefinitions.h"
#include "../../Robot.h"

ArmMoveUp::ArmMoveUp() {
	Requires(Robot::cubeManipulation.get());
}

// Called just before this Command runs the first time
void ArmMoveUp::Initialize() {
	Robot::cubeManipulation->SetShoulderPower(0);
}

// Called repeatedly when this Command is scheduled to run
void ArmMoveUp::Execute() {

	std::shared_ptr<Joystick> joystick = Robot::oi->getauxStick();
	const double rightYAxisValue = joystick->GetRawAxis(XBox_RightYAxis);

	// If it's far enough away from the mid-point of the joystick, then
	// we'll treat it as a real power setting.  Otherwise, it's in the
	// "dead zone", and we'll bring the power to 0.
	if (fabs(rightYAxisValue) > .05f) {
		Robot::cubeManipulation->SetShoulderPower(rightYAxisValue);
	} else {
		Robot::cubeManipulation->SetShoulderPower(0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ArmMoveUp::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ArmMoveUp::End() {
	Robot::cubeManipulation->SetShoulderPower(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ArmMoveUp::Interrupted() {
	End();
}
