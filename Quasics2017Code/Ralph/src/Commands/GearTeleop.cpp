#include "GearTeleop.h"
#include "../Robot.h"

GearTeleop::GearTeleop() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::gear.get());
	previousValue = false;
}

// Called just before this Command runs the first time
void GearTeleop::Initialize() {
	previousValue = false;
}

// Called repeatedly when this Command is scheduled to run
void GearTeleop::Execute() {
	if(!Robot::oi->getAuxStick()->GetRawButton(GearActuatorButton) && previousValue){
		Robot::gear->Set(!Robot::gear->Get());
	}

	previousValue = Robot::oi->getAuxStick()->GetRawButton(GearActuatorButton);
}

// Make this return true when this Command no longer needs to run execute()
bool GearTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GearTeleop::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearTeleop::Interrupted() {

}
