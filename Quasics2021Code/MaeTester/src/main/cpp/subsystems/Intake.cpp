// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"

// #define LOG_LIMIT_SWITCH_STATE

Intake::Intake()
    : intakeMotor(CANBusIds::VictorSPXIds::IntakeMotor),
      conveyorMotor(CANBusIds::VictorSPXIds::ConveyorMotor) {
  conveyorBeamSensor.reset(
      new frc::DigitalInput(DigitalIOMappings::ConveyorBeamSensor));
  SetSubsystem("Intake");
}

// This method will be called once per scheduler run
void Intake::Periodic() {
#ifdef LOG_LIMIT_SWITCH_STATE
  std::cout << "Limit switch is " << (conveyorLimitSwitch->Get() ? "" : "not ")
            << "open" << std::endl;
#endif

#ifdef LOG_BEAM_SENSOR_STATE
  std::cout << "Beam sensor is " << (conveyorBeamSensor->Get() ? "" : "not ")
            << "open" << std::endl;
#endif
}

// Intakes the ball at 1/4 speed. Theoretically, when limit switch is hit, both
// Intake and Conveyor will stop.
void Intake::IntakeBallOn() {
  if (!IsBallInChamber()) {
    conveyorMotor.Set(0.75);
    intakeMotor.Set(0.25);
  } else {
    intakeMotor.Set(0);
    conveyorMotor.Set(0);
  }
}
// Automatic Intake: if limit switch is hit, only conveyor will disable.
// Otherwise, will always run.
void Intake::IntakeCellsAuto() {
  intakeMotor.Set(0.75);
  if (conveyorBeamSensor.get()) {
    conveyorMotor.Set(0.25);
  } else {
    conveyorMotor.Set(0);
  }
}

// Stops the intake of ball.
void Intake::IntakeBallOff() {
  intakeMotor.Set(0);
  conveyorMotor.Set(0);
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
  conveyorMotor.Set(1.0);
}

void Intake::ConveyBallReverse() {
  conveyorMotor.Set(-.25);
}

void Intake::ConveyBallOff() {
  conveyorMotor.Set(0);
}

bool Intake::IsBallInChamber() {
  //return !conveyorLimitSwitch->Get();
  return !conveyorBeamSensor->Get();
}
