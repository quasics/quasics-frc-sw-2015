#include "ShoulderMove.h"
#include "../../Robot.h"

ShoulderMove::ShoulderMove(double seconds, double power): frc::Command() {
	count = 0;
	m_seconds = seconds;
	m_power = power;
	Requires (Robot::cubeManipulation.get());
}

// Called just before this Command runs the first time
void ShoulderMove::Initialize() {
	Robot::cubeManipulation->SetShoulderPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void ShoulderMove::Execute() {
	while(count <= m_seconds){
		count = count + 1;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ShoulderMove::IsFinished() {
	return count >= m_seconds;
}

// Called once after isFinished returns true
void ShoulderMove::End() {
	Robot::cubeManipulation->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShoulderMove::Interrupted() {
	End();
}
