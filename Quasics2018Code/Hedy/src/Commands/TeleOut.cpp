#include "TeleOut.h"
#include "../Robot.h"
#include "../ControllerDefinitions.h"


TeleOut::TeleOut(): frc::Command() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void TeleOut::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TeleOut::Execute() {
	std::shared_ptr<Joystick> joystick = Robot::oi->getauxStick();
	const bool high = joystick->GetRawButton(XBox_ButtonB);
	const bool low = joystick->GetRawButton(XBox_ButtonY);

	if(high){
		Robot::cubeManipulation->SetIntakePower(.6);
	}
	else if(low){
		Robot::cubeManipulation->SetIntakePower(.4);
	}
	else{
		Robot::cubeManipulation->SetIntakePower(0);
	}

}

// Make this return true when this Command no longer needs to run execute()
bool TeleOut::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TeleOut::End() {
	Robot::cubeManipulation->SetIntakePower(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TeleOut::Interrupted() {
	Robot::cubeManipulation->SetIntakePower(0);
}
