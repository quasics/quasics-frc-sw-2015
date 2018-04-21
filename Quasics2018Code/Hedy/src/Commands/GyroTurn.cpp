#include "GyroTurn.h"
#include "../Robot.h"

#include <iostream>
#include <iomanip>

double normalizeAngle(double angleDegrees) {
	double result = 0;
	if (angleDegrees >= 0 && angleDegrees < 360) {
		result = angleDegrees;
	} else if (angleDegrees < 0) {
		// Angle is negative (and for now, we'll only turn to the left), so
		// convert it.
		result = angleDegrees - (((int(angleDegrees) / 360) - 1) * 360);
	} else {
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

// Called just before this Command runs the first time
void GyroTurn::Initialize() {
	std::cerr << "Initializing GyroTurn(" << m_angle << ", " << m_power << ")\n";
	Robot::gyroADXRS->Calibrate();
	Robot::gyroADXRS->Reset();
	Robot::driveBase->SetPowerToMotors(m_power, m_power);
	SmartDashboard::PutString("DB/String 0", "Code has passed calibrate, reset, and motors.");
}

// Called repeatedly when this Command is scheduled to run
void GyroTurn::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool GyroTurn::IsFinished() {
	const double currentAngle = Robot::gyroADXRS->GetAngle();
	std::cerr << "Target angle: " << std::setw(6) << m_angle
			  << "\tCurrent angle: " << std::setw(6) << currentAngle
			  << std::endl;

	if (currentAngle >= m_angle){
		return true;
	}
	return false;
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
