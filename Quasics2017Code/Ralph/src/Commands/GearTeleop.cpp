#include "GearTeleop.h"
#include "../Robot.h"
#include "../RobotVariables.h"
#include "iostream"

GearTeleop::GearTeleop() : previousValue(false) {
	Requires(Robot::gear.get());
}

void GearTeleop::Initialize() {
	previousValue = false;
}

// CODE_REVIEW(mjh): Document what this #define is used for, and why it should
// be enabled (or disabled).
//#define DSGearTeleop

// Called repeatedly when this Command is scheduled to run
void GearTeleop::Execute() {
#ifndef DSGearTeleop
	if(!Robot::oi->getAuxStick()->GetRawButton(GearActuatorButton) && previousValue) {
		Robot::gear->Set(!Robot::gear->Get());
	}

	previousValue = Robot::oi->getAuxStick()->GetRawButton(GearActuatorButton);
#else
	Robot::gear->Set(!Robot::gear->Get());
#endif
}

// TODO: Make this return true when this Command no longer needs to run execute()
bool GearTeleop::IsFinished() {
#ifndef DSGearTeleop
	return false;
#else
	return true;
#endif
}
