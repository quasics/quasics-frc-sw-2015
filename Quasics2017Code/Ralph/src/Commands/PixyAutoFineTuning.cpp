#include "PixyAutoFineTuning.h"
#include "../Robot.h"

PixyAutoFineTuning::PixyAutoFineTuning() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::arduino.get());
	Requires(Robot::driveTrain.get());
	Requires(Robot::gear.get());

	timer = 0;
	isFarLeft = false;
	isAligned = false;
	isTooFar = false;
}

// Called just before this Command runs the first time
void PixyAutoFineTuning::Initialize() {
	//Todo: Add in a one-time NeoPixel set
	Robot::arduino->SetBrightnessMode(Arduino::kBlinking);
	Robot::arduino->SetLEDColor(Arduino::kTeam);
	timer = 0;
	isFarLeft = false;
	isAligned = false;
	isTooFar = false;
}

// Called repeatedly when this Command is scheduled to run
void PixyAutoFineTuning::Execute() {

	bool gotData = Robot::arduino->GetCameraData(isFarLeft, isAligned,
			isTooFar);
	if (gotData) {
		if (isTooFar) {
			if (isAligned) {
				Robot::driveTrain->SetLeftPower(.25);
				Robot::driveTrain->SetRightPower(.25);
			} else {
				if (isFarLeft) {
					Robot::driveTrain->SetLeftPower(.375);
					Robot::driveTrain->SetRightPower(.25);
				} else {
					Robot::driveTrain->SetLeftPower(.25);
					Robot::driveTrain->SetRightPower(.375);
				}
			}
		} else {
			if (isAligned) {
				Robot::driveTrain->SetLeftPower(0);
				Robot::driveTrain->SetRightPower(0);
			} else {
				if (isFarLeft) {
					Robot::driveTrain->SetLeftPower(.25);
					Robot::driveTrain->SetRightPower(-.25);
				} else {
					Robot::driveTrain->SetLeftPower(-.25);
					Robot::driveTrain->SetRightPower(.25);
				}
			}
		}
	}
	timer++;
}

// Make this return true when this Command no longer needs to run execute()
bool PixyAutoFineTuning::IsFinished() {
	return timer >= uint32_t(AutoTimeOut * 50 + .5)
			|| (isFarLeft && isAligned && isTooFar);
}

// Called once after isFinished returns true
void PixyAutoFineTuning::End() {
	Robot::driveTrain->SetLeftPower(0);
	Robot::driveTrain->SetRightPower(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PixyAutoFineTuning::Interrupted() {
	Robot::driveTrain->SetLeftPower(0);
	Robot::driveTrain->SetRightPower(0);

}
