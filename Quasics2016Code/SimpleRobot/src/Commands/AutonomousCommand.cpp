// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include "AutonomousCommand.h"

#include <iostream>
#include "../Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousCommand::AutonomousCommand(int TargetPosition)
	: AutonomousCommand(FieldPosition(TargetPosition))
{
}

AutonomousCommand::AutonomousCommand(FieldPosition targetPosition)
  : m_TargetPosition(targetPosition), isEndstep(false),
	autoStage(0), endstepTimer(0)
{
	// Use requires() here to declare subsystem dependencies
	Requires(Robot::driveSystem.get());

	Robot::driveSystem->ResetEncoders();
	Robot::driveSystem->ResetYaw();
}

// Called just before this Command runs the first time
void AutonomousCommand::Initialize() {
	isEndstep = false;
	autoStage = 0;
	endstepTimer = 0;

	Robot::driveSystem->ResetEncoders();
	Robot::driveSystem->ResetYaw();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() {
	if (!isEndstep) {
		switch (m_TargetPosition) {
		case ePosition1:
			isEndstep = Auto1();
			break;

		case ePosition2:
			isEndstep = Auto2();
			break;

		case ePosition3:
			isEndstep = Auto3();
			break;

		case ePosition4:
			isEndstep = Auto4();
			break;

		case ePosition5:
			isEndstep = Auto5();
			break;

		case ePositionNull:		// Do nothing!
		default:
			isEndstep = true;
			break;
		}
	} else if (endstepTimer <= 50){
		Robot::intakeArm->SetArmDirection(IntakeArm::kFalling);
		Robot::driveSystem->StopEverything();
		endstepTimer++;
	} else {
		Robot::intake->SetPower(Intake::kOutput);
		Robot::driveSystem->StopEverything();
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutonomousCommand::End() {
	Robot::intake->StopIntake();
	Robot::intakeArm->StopArm();
	Robot::driveSystem->StopEverything();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousCommand::Interrupted() {
	Robot::intake->StopIntake();
	Robot::intakeArm->StopArm();
	Robot::driveSystem->StopEverything();
}

