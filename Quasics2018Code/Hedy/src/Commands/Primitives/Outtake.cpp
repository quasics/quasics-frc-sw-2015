#include "Outtake.h"
#include "../../Robot.h"

Outtake::Outtake(double seconds, double power): frc::Command() {
	count = 0;
	m_ticksToRun = seconds * 50;		// Because Execute is called at 50Hz
	m_power = power;
	Requires (Robot::cubeIntake.get());
}

// Called just before this Command runs the first time
void Outtake::Initialize() {
	Robot::cubeIntake->SetIntakePower(-m_power);
}

// Called repeatedly when this Command is scheduled to run
void Outtake::Execute() {
	count = count + 1;
}

// Make this return true when this Command no longer needs to run execute()
bool Outtake::IsFinished() {
	return count >= m_ticksToRun;
}

// Called once after isFinished returns true
void Outtake::End() {
	Robot::cubeIntake->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Outtake::Interrupted() {
	End();
}
