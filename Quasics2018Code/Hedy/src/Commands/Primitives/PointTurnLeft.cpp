#include "PointTurnLeft.h"
#include "../../Robot.h"

PointTurnLeft::PointTurnLeft(double angle, double power): frc::Command() {

	m_angle = angle;		// because Execute() is run at 50Hz
	m_power = power;
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void PointTurnLeft::Initialize() {
	//robot turns left
	Robot::driveBase->SetPowerToMotors(m_power, -m_power);

}

// Called repeatedly when this Command is scheduled to run
void PointTurnLeft::Execute() {
	std::cout << "Angle: " << Robot::navigation->getAngle() << " Wanted Angle: " << m_power << "\n";
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnLeft::IsFinished() {
	if (Robot::navigation->getAngle() >= m_angle) {
			return true;
		}
		else {
			return false;
		}
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
