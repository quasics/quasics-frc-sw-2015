

#include "PneumaticMove.h"
PneumaticMove::PneumaticMove(): frc::Command() {
	//add requires pneumatics subsystem


}

// Called just before this Command runs the first time
void PneumaticMove::Initialize() {
	frc::DoubleSolenoid rampDouble {1,2};
	rampDouble.Set(frc::DoubleSolenoid::Value::kOff);
	rampDouble.Set(frc::DoubleSolenoid::Value::kForward);
	rampDouble.Set(frc::DoubleSolenoid::Value::kReverse);

}

// Called repeatedly when this Command is scheduled to run
void PneumaticMove::Execute() {


}

// Make this return true when this Command no longer needs to run execute()
bool PneumaticMove::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void PneumaticMove::End() {


}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PneumaticMove::Interrupted() {

}
