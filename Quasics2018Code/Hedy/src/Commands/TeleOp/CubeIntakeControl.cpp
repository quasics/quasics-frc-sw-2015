#include <Commands/TeleOp/CubeIntakeControl.h>
#include "../../ControllerDefinitions.h"
#include "../../Robot.h"
#include "../../Subsystems/CubeIntake.h"

CubeIntakeControl::CubeIntakeControl() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::cubeIntake.get());
}

// Called just before this Command runs the first time
void CubeIntakeControl::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void CubeIntakeControl::Execute() {

	std::shared_ptr<Joystick> joystick = Robot::oi->getauxStick();
	const bool buttonA = joystick->GetRawButton(XBox_ButtonA);
	const bool buttonB = joystick->GetRawButton(XBox_ButtonB);

	if(buttonA && !Robot::cubeIntake->IsLimitSwitchTriggered()){
		Robot::cubeIntake->SetIntakePower(.40);
	}
	else if (buttonB) {
		Robot::cubeIntake->SetIntakePower(-.40);
	}
	else{
		Robot::cubeIntake->SetIntakePower(0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool CubeIntakeControl::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void CubeIntakeControl::End() {
	Robot::cubeIntake->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CubeIntakeControl::Interrupted() {
	End();
}
