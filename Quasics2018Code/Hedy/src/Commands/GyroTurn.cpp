#include <Commands/GyroTurn.h>
#include "../Robot.h"

GyroTurn::GyroTurn(float angle, double power): frc::Command() {
	Requires (Robot::gyroADXRS.get());

	m_angle = angle;
	m_power = power;
}

// Called just before this Command runs the first time
void GyroTurn::Initialize() {
	Robot::gyroADXRS->Reset();
	Robot::driveBase->SetPowerToMotors(m_power, m_power);
}

// Called repeatedly when this Command is scheduled to run
void GyroTurn::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool GyroTurn::IsFinished() {
	if (Robot::gyroADXRS->GetAngle() >= m_angle){
		return true;
	}
	else if(Robot::gyroADXRS->GetAngle() < 0){
		m_angle = (int(Robot::gyroADXRS->GetAngle()) / 360 + 1) * 360 + Robot::gyroADXRS->GetAngle();

	}
	else{
		m_angle = Robot::gyroADXRS->GetAngle() - ((int(Robot::gyroADXRS->GetAngle()) / 360) * 360);
	}

}

// Called once after isFinished returns true
void GyroTurn::End() {
	Robot::driveBase->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GyroTurn::Interrupted() {
	End();
}
