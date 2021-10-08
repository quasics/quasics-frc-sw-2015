// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"
#include "LoggingUtils.h"

// #define LOG_LIMIT_SWITCH_STATE

constexpr double MOTOR_OFF_POWER = 0;
constexpr double MOTOR_SLOW_POWER = 0.25;
constexpr double MOTOR_FAST_POWER = 0.75;
constexpr double MOTOR_FULL_POWER = 1.0;

Intake::Intake()
    : intakeMotor(CANBusIds::VictorSPXIds::IntakeMotor),
      conveyorMotor(CANBusIds::VictorSPXIds::ConveyorMotor) {
#if defined(INTAKE_USES_LIMIT_SWITCH)
  conveyorLimitSwitch.reset(
      new frc::DigitalInput(DigitalIOMappings::IntakeLimitSwitch));
#elif defined(INTAKE_USES_BEAM_SENSOR)
  conveyorBeamSensor.reset(
      new frc::DigitalInput(DigitalIOMappings::ConveyorBeamSensor));
#endif
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
    conveyorMotor.Set(MOTOR_FAST_POWER);
    intakeMotor.Set(MOTOR_SLOW_POWER);
  } else {
    intakeMotor.Set(MOTOR_OFF_POWER);
    conveyorMotor.Set(MOTOR_OFF_POWER);
  }
}
// Automatic Intake: if limit switch is hit, only conveyor will disable.
// Otherwise, will always run.
// TODO: Fix this: it shouldn't be written like this!!!!!
// TODO(matt): Explain to people why.
void Intake::IntakeCellsAuto() {
  intakeMotor.Set(MOTOR_FAST_POWER);
  if (IsBallInChamber()) {
    conveyorMotor.Set(MOTOR_SLOW_POWER);
  } else {
    conveyorMotor.Set(MOTOR_OFF_POWER);
  }
}

// Stops the intake of ball.
void Intake::IntakeBallOff() {
  intakeMotor.Set(MOTOR_OFF_POWER);
  conveyorMotor.Set(MOTOR_OFF_POWER);
}

void Intake::ConveyBallOn() {
  conveyorMotor.Set(MOTOR_FULL_POWER);
}

void Intake::ConveyBallReverse() {
  conveyorMotor.Set(-MOTOR_SLOW_POWER);
}

void Intake::ConveyBallOff() {
  conveyorMotor.Set(MOTOR_OFF_POWER);
}

void Intake::OnlyIntakeOn() {
  intakeMotor.Set(MOTOR_FAST_POWER);
}

void Intake::OnlyIntakeReverse() {
  intakeMotor.Set(-MOTOR_FAST_POWER);
}

void Intake::OnlyIntakeOff() {
  intakeMotor.Set(MOTOR_OFF_POWER);
}

bool Intake::IsBallInChamber() {
#if defined(INTAKE_USES_LIMIT_SWITCH)
  return !conveyorLimitSwitch->Get();
#elif defined INTAKE_USES_BEAM_SENSOR
  return !conveyorBeamSensor->Get();
#else
  LOG_EVERY_N_TIMES(50,
                    "Checking for ball in chamber, but no sensor is enabled!");
  return false;
#endif
}
