// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"

#define LOG_LIMIT_SWITCH_STATE

Intake::Intake()
    : intakeMotor(CANBusIds::VictorSPXIds::IntakeMotor),
      conveyorMotor(CANBusIds::VictorSPXIds::ConveyorMotor) {
  conveyorLimitSwitch.reset(
      new frc::DigitalInput(DigitalIOMappings::IntakeLimitSwitch));
  SetSubsystem("Intake");
}

// This method will be called once per scheduler run
void Intake::Periodic() {
#ifdef LOG_LIMIT_SWITCH_STATE
  std::cout << "Limit switch is " << (conveyorLimitSwitch->Get() ? "" : "not ")
            << "set" << std::endl;
#endif
}

// Intakes the ball at 1/4 speed. Theoretically, when limit switch is hit, both
// Intake and Conveyor will stop.
void Intake::IntakeBallOn() {
  if (conveyorLimitSwitch.get()) {
    conveyorMotor.Set(0);
    intakeMotor.Set(0);
  } else {
    intakeMotor.Set(0.75);
    conveyorMotor.Set(0.25);
  }

//Stops the intake of ball.
}
void Intake::IntakeBallOff() {
  intakeMotor.Set(0);
  conveyorMotor.Set(0);

  // Reverses the intake and conveyor at 1/4 speed.
}


void Intake::OnlyIntakeOn() {
  intakeMotor.Set(.75);
}

void Intake::OnlyIntakeReverse() {
  intakeMotor.Set(-.75);
}

void Intake::OnlyIntakeOff() {
  intakeMotor.Set(0);
}

void Intake::ConveyBallOn() {
  conveyorMotor.Set(.25);
}

void Intake::ConveyBallReverse() {
  conveyorMotor.Set(-.25);
}

void Intake::ConveyBallOff() {
  conveyorMotor.Set(0);
}



