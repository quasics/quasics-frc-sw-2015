#include "PointTurnLeft.h"
#include "../../Robot.h"

PointTurnLeft::PointTurnLeft(double angle, double power): frc::Command() {
	roboAngle = 0;
	m_angle = angle;		// because Execute() is run at 50Hz
	m_power = power;
	Requires (Robot::driveBase.get());
	Requires (Robot::navigation.get());
}

// Called just before this Command runs the first time
void PointTurnLeft::Initialize() {
	//robot turns left
	Robot::driveBase->SetPowerToMotors(m_power, m_power);
	Robot::navigation->resetBearing();
}

// Called repeatedly when this Command is scheduled to run
void PointTurnLeft::Execute() {
	roboAngle = fabs(Robot::navigation->getBearing());
	if(Robot::navigation->getBearing()  < m_angle/2){
		Robot::driveBase->SetPowerToMotors(m_power, m_power);
	}else{
		Robot::driveBase->SetPowerToMotors(m_power / 2, m_power / 2);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnLeft::IsFinished() {
	return (Robot::navigation->getBearing() > m_angle);
}

// Called once after isFinished returns true
void PointTurnLeft::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnLeft::Interrupted() {
	End();
}
