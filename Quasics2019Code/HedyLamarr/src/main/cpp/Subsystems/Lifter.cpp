#ifdef ENABLE_OLD_ELEVATOR
// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Subsystems/Lifter.h"

#include <iostream>
#include "ConfigurationFlags.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Lifter::Lifter() : ElevatorStage("Lifter") {
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
 // hallEffectHighLifter.reset(new frc::DigitalInput(4));     // TODO: RE-ENABLE THIS, IF ELEVATOR SWITCHES BACK TO PORT 8 FOR BOTTOM LIMIT SWITCH!!!
  hallEffectLowLifter.reset(new frc::DigitalInput(3));
  lifterMotor.reset(new frc::Spark(6));
  lifterFirmStopHigh.reset(new frc::DigitalInput(5));
  lifterFirmStopLow.reset(new frc::DigitalInput(6));
  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  lifterMotor->SetInverted(false);
}

bool Lifter::atTop() {
#ifdef DISABLE_ELEVATOR_LIMIT_SWITCHES
  return false;
#else
  return lifterFirmStopHigh->Get();
#endif
}

bool Lifter::atBottom() {
#ifdef DISABLE_ELEVATOR_LIMIT_SWITCHES
  return false;
#else
  return lifterFirmStopLow->Get();
#endif
}

bool Lifter::atPositionOne() {
  return hallEffectLowLifter->Get();
}

bool Lifter::atPositionTwo() {
  return hallEffectHighLifter->Get();
}

void Lifter::moveUp() {
  lifterMotor->Set(-.8);
}

void Lifter::moveDown() {
  lifterMotor->Set(.65);
}

void Lifter::moveSlowlyUp() {
  lifterMotor->Set(-.8);
}

void Lifter::moveSlowlyDown() {
  lifterMotor->Set(.65);
}

void Lifter::stop() {
  lifterMotor->Set(0.0);
}
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.
#endif // ENABLE_OLD_ELEVATOR