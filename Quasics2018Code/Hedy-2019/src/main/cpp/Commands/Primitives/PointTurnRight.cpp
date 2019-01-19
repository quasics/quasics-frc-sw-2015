#include "PointTurnRight.h"
#include "../../Robot.h"

PointTurnRight::PointTurnRight(double angle, double power): frc::Command() {
	m_angle = angle;		// because Execute() is run at 50Hz
	m_power = power;
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void PointTurnRight::Initialize() {
	//robot turns right

	Robot::driveBase->SetPowerToMotors(-m_power, -m_power);
}

// Called repeatedly when this Command is scheduled to run
void PointTurnRight::Execute() {
	if(Robot::navigation->getBearing()  < m_angle/2){
		Robot::driveBase->SetPowerToMotors(m_power, m_power);
	}else{
		Robot::driveBase->SetPowerToMotors(m_power / 2, m_power / 2);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnRight::IsFinished() {
	return (Robot::navigation->getBearing() > m_angle);
}

// Called once after isFinished returns true
void PointTurnRight::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnRight::Interrupted() {
	End();
}
