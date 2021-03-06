// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Intake.h"
#include "../RobotMap.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Intake::Intake() :
		Subsystem("Intake") {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	leftIntakeWheel = RobotMap::intakeLeftIntakeWheel;
	rightIntakeWheel = RobotMap::intakeRightIntakeWheel;
	pusher = RobotMap::intakePusher;
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	leftIntakeWheel->SetInverted(true);
}

void Intake::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Intake::SetPower(Direction whichDirection) {
	switch (whichDirection) {
	case kIntake:
		leftIntakeWheel->Set(1);
		rightIntakeWheel->Set(1);
		break;
	case kOutput:
		leftIntakeWheel->Set(-1);
		rightIntakeWheel->Set(-1);
		break;
	default:
		leftIntakeWheel->Set(0);
		rightIntakeWheel->Set(0);
		break;
	}
}

void Intake::PistonControl(bool isOn) {
	static bool previousSetting = false;
	static int stateTimer = 0;
	if (isOn && !previousSetting) {
		pusher->Set(DoubleSolenoid::kForward);
		stateTimer = 0;
	} else if (isOn && stateTimer <= 25){
		pusher->Set(DoubleSolenoid::kForward);
		stateTimer++;
	} else if (!isOn && previousSetting){
		pusher->Set(DoubleSolenoid::kReverse);
		stateTimer = 0;
	} else if (stateTimer <= 25) {
		pusher->Set(DoubleSolenoid::kReverse);
		stateTimer++;
	} else {
		pusher->Set(DoubleSolenoid::kOff);
		stateTimer++;
	}

	previousSetting = isOn;
}

void Intake::StopIntake() {
	leftIntakeWheel->Set(0);
	rightIntakeWheel->Set(0);
	pusher->Set(DoubleSolenoid::kOff);
}
