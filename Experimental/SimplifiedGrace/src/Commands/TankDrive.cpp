#include "TankDrive.h"
#include "../Robot.h"
#include "../ControllerDefinitions.h"


TankDrive::TankDrive() {
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
}

void TankDrive::Initialize(){
}

void TankDrive::Execute(){
	std::shared_ptr<Joystick> joystick = Robot::oi->getDriveStick();
	const bool leftTurbo = joystick->GetRawButton(LogitechGamePad_LeftTrigger);
	const bool rightTurbo = joystick->GetRawButton(LogitechGamePad_RightTrigger);
	double mult=.4;
	if(leftTurbo && rightTurbo){
		mult = .7;
	}
	const bool buttonIsPressedNow = joystick->GetRawButton(LogitechGamePad_AButton);
	if(!buttonIsPressedNow && pressedLastTime){
		counter = counter + 1;
	}
	if (counter%2 == 0){
	double leftY_value = joystick->GetRawAxis(LogitechGamePad_LeftYAxis)* -mult;
	double rightY_value = joystick->GetRawAxis(LogitechGamePad_RightYAxis)* -mult;

	Robot::driveBase->SetLeftPower(leftY_value);
	Robot::driveBase->SetRightPower(rightY_value);
	}
	else{
		double leftY_value = joystick->GetRawAxis(LogitechGamePad_LeftYAxis)* mult;
		double rightY_value = joystick->GetRawAxis(LogitechGamePad_RightYAxis)* mult;

		Robot::driveBase->SetLeftPower(rightY_value);
		Robot::driveBase->SetRightPower(leftY_value);
	}

	pressedLastTime = buttonIsPressedNow;
}

bool TankDrive::IsFinished(){
	return false;
}

void TankDrive::End(){
	Robot::driveBase->Stop();
}

void TankDrive::Interrupted(){
	Robot::driveBase->Stop();
}
