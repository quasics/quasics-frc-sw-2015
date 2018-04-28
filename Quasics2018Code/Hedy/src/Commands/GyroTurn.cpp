#include "GyroTurn.h"
#include "../Robot.h"

#include <iostream>
#include <iomanip>

#define NOISY

double normalizeAngle(double angleDegrees) {

	double result = 0;
	if (angleDegrees < 360) {
		result = angleDegrees;
	}

	else if (angleDegrees >= 361) {
		// Angle is too big: reduce it
		result = angleDegrees - ((int(angleDegrees) / 360) * 360);
	}
	return result;
}

GyroTurn::GyroTurn(float angle, double power): frc::Command() {
	Requires (Robot::gyroADXRS.get());

	m_angle = normalizeAngle(angle);
	m_power = power;
}

void GyroTurn::DumpStats(std::string prefix) {
#ifdef NOISY
	const double turnRate = Robot::gyroADXRS->GetRate();
	const double currentAngle = Robot::gyroADXRS->GetAngle();
	std::cerr << prefix
			  << "Target angle: " << std::setw(6) << m_angle
			  << "\tCurrent angle: " << std::setw(6) << currentAngle
			  << "\tCurrent rate: " << std::setw(6) << turnRate
			  << std::endl;
#endif	// NOISY
}

// Called just before this Command runs the first time
void GyroTurn::Initialize() {
	std::cerr << "Initializing GyroTurn(" << m_angle << ", " << m_power << ")\n";
	Robot::gyroADXRS->Reset();

	DumpStats("At init: ");

	Robot::driveBase->SetPowerToMotors(m_power, m_power);
}

// Called repeatedly when this Command is scheduled to run
void GyroTurn::Execute() {

	const double currentAngle = Robot::gyroADXRS->GetAngle();
	if (currentAngle + 10 >= m_angle && m_angle > 0){
		Robot::driveBase->SetPowerToMotors(.15, .15);

	}
}

// Make this return true when this Command no longer needs to run execute()
bool GyroTurn::IsFinished() {
	DumpStats();
	const double currentAngle = Robot::gyroADXRS->GetAngle();
	if (currentAngle >= m_angle && m_angle > 0){
		return true;
	}
	else if (m_angle < 0 && currentAngle <= m_angle){
		return true;

	}
	return false;

}

// Called once after isFinished returns true
void GyroTurn::End() {
	Robot::driveBase->Stop();

	DumpStats("At end: ");
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GyroTurn::Interrupted() {
	End();
}
