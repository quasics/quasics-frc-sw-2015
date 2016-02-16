// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "ArcadeDrive.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ArcadeDrive::ArcadeDrive(): Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveSystem.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void ArcadeDrive::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
	float mult;
	if ((Robot::oi->getPilotStick()->GetRawButton(5)
			|| Robot::oi->getPilotStick()->GetRawButton(6))
			&& !(Robot::oi->getPilotStick()->GetRawButton(7)
					|| Robot::oi->getPilotStick()->GetRawButton(8)))
		mult = .25;
	else if (!(Robot::oi->getPilotStick()->GetRawButton(5)
			|| Robot::oi->getPilotStick()->GetRawButton(6))
			&& (Robot::oi->getPilotStick()->GetRawButton(7)
					|| Robot::oi->getPilotStick()->GetRawButton(8)))
		mult = .625;
	else
		mult = .5;

	float yAxis;
	float xAxis;


	if (fabs(Robot::oi->getPilotStick()->GetRawAxis(1)) >=.05)
		yAxis = Robot::oi->getPilotStick()->GetRawAxis(1);
	else
		yAxis = 0;

	if (fabs(Robot::oi->getPilotStick()->GetRawAxis(0)) >=.05)
		xAxis = Robot::oi->getPilotStick()->GetRawAxis(0);
	else
		xAxis = 0;

	Robot::driveSystem->MoveLeft(((yAxis * 100) - (xAxis * 100)) * mult);
	Robot::driveSystem->MoveRight(((yAxis * 100) + (xAxis * 100)) * mult);
}

// Make this return true when this Command no longer needs to run execute()
bool ArcadeDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ArcadeDrive::End() {
	Robot::driveSystem->StopEverything();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ArcadeDrive::Interrupted() {
	Robot::driveSystem->StopEverything();
}
