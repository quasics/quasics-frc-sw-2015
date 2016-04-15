// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "TwoPersonIntake.h"

#include <Commands/Subsystem.h>
#include "../Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

TwoPersonIntake::TwoPersonIntake() :
		Command() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::intake.get());
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void TwoPersonIntake::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TwoPersonIntake::Execute() {
	bool isMoving = false;

	if (Robot::oi->getIntakeStick()->GetRawButton(1)
			&& !Robot::oi->getIntakeStick()->GetRawButton(3)) {
		Robot::intake->SetPower(Intake::kOutput);
		isMoving = true;
	} else if (!Robot::oi->getIntakeStick()->GetRawButton(1)
			&& Robot::oi->getIntakeStick()->GetRawButton(3)) {
		Robot::intake->SetPower(Intake::kIntake);
		isMoving = true;
	} else {
		Robot::intake->StopIntake();
		isMoving = false;
	}
		Robot::intake->PistonControl(Robot::oi->getIntakeStick()->GetRawButton(2));
}

// Make this return true when this Command no longer needs to run execute()
bool TwoPersonIntake::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TwoPersonIntake::End() {
	Robot::intake->StopIntake();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TwoPersonIntake::Interrupted() {
	Robot::intake->StopIntake();
}
