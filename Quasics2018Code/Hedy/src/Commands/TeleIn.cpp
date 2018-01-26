#include "TeleIn.h"
#include "../ControllerDefinitions.h"
#include "../Robot.h"


TeleIn::TeleIn() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void TeleIn::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TeleIn::Execute() {

	std::shared_ptr<Joystick> joystick = Robot::oi->getauxStick();
	const bool buttonA = joystick->GetRawButton(XBox_ButtonA);

		if(buttonA){
			Robot::cubeManipulation->SetIntakePower(.6);
			}
		else{
			Robot::cubeManipulation->SetIntakePower(0);
		}
}

// Make this return true when this Command no longer needs to run execute()
bool TeleIn::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TeleIn::End() {
	Robot::driveBase->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TeleIn::Interrupted() {
	Robot::driveBase->Stop();

}
