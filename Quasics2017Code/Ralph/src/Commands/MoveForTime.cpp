// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "MoveForTime.h"
#include "../Robot.h"

MoveForTime::MoveForTime(double seconds, double power): Command() {
    m_seconds = seconds;
    m_power = power;
	Requires(Robot::driveTrain.get());
	counter = 0;
}

void MoveForTime::Initialize() {
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
	counter = 0;

}

// Called repeatedly when this Command is scheduled to run
void MoveForTime::Execute() {
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForTime::IsFinished() {
    return counter >= m_seconds * 50;
}

// Called once after isFinished returns true
void MoveForTime::End() {
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForTime::Interrupted() {
	Robot::driveTrain->Stop();
}
