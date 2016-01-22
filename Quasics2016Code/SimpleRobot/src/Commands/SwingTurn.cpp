// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "SwingTurn.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

SwingTurn::SwingTurn(double powerPercent, double degrees, bool turnLeft): Command() {
    m_powerPercent = powerPercent;
    m_degrees = degrees;
    m_turnLeft = turnLeft;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveSystem.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void SwingTurn::Initialize() {
	Robot::driveSystem->ResetYaw();
	if(m_turnLeft){
			Robot::driveSystem->MoveLeft(0);
			Robot::driveSystem->MoveRight(m_powerPercent);
	}
	else{
			Robot::driveSystem->MoveLeft(m_powerPercent);
			Robot::driveSystem->MoveRight(0);
	}
}

// Called repeatedly when this Command is scheduled to run
void SwingTurn::Execute() {
	if(m_turnLeft){
			Robot::driveSystem->MoveLeft(0);
			Robot::driveSystem->MoveRight(m_powerPercent);
	}
	else{
			Robot::driveSystem->MoveLeft(m_powerPercent);
			Robot::driveSystem->MoveRight(0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool SwingTurn::IsFinished() {
    return (fabs(Robot::driveSystem->GetContinuousYaw()) >= fabs(m_degrees));
}

// Called once after isFinished returns true
void SwingTurn::End() {
	Robot::driveSystem->StopEverything();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwingTurn::Interrupted() {
	Robot::driveSystem->StopEverything();
}
