

#include "TankDrive.h"
#include "../../Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

TankDrive::TankDrive(): frc::Command() {

	Requires(Robot::driveBase.get());

}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
	std::shared_ptr<Joystick> joystick = Robot::oi->getdriveStick();
	const bool highBoost = Robot::oi->isHighBoostSignaled();
	const bool lowBoost = Robot::oi->isLowBoostSignaled();
	const bool switchDirection = Robot::oi->isSwitchDirectionSignaled();
	const double leftTrackPower = Robot::oi->getLeftTrackPower();
	const double rightTrackPower = Robot::oi->getRightTrackPower();

	double mult = .4;
	if(highBoost && lowBoost){
		mult = 1;
	}
	else if(highBoost && !lowBoost){
		mult = .6;
	}
	else if(lowBoost && !highBoost){
		mult = .4;
	}

	if(!switchDirection && pressedLastTime){
		counter = counter + 1;
	}


	if(counter % 2 == 0) {
		Robot::driveBase->SetPowerToMotors(leftTrackPower * -mult, (rightTrackPower * mult));
	}
	else {
		Robot::driveBase->SetPowerToMotors(leftTrackPower * mult, -(rightTrackPower * mult));
	}
}

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void TankDrive::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TankDrive::Interrupted() {
	End();
}
