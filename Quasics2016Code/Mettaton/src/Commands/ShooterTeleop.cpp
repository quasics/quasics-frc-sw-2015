// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "ShooterTeleop.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ShooterTeleop::ShooterTeleop() :
		Command() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::shooter.get());
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void ShooterTeleop::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ShooterTeleop::Execute() {
	if (Robot::oi->getShooterStick()->GetRawButton(1)
			&& !Robot::oi->getShooterStick()->GetRawButton(3)) {
		Robot::shooter->SetWheels(true);
	} else if (!Robot::oi->getShooterStick()->GetRawButton(1)
			&& Robot::oi->getShooterStick()->GetRawButton(3)) {
		Robot::shooter->SetWheels(false);
	} else {
		Robot::shooter->StopWheels();
	}
	Robot::shooter->SetPiston(Robot::oi->getShooterStick()->GetRawButton(2));
}

// Make this return true when this Command no longer needs to run execute()
bool ShooterTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ShooterTeleop::End() {
	Robot::shooter->StopWheels();
	Robot::shooter->StopPiston();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShooterTeleop::Interrupted() {
	Robot::shooter->StopWheels();
	Robot::shooter->StopPiston();
}
