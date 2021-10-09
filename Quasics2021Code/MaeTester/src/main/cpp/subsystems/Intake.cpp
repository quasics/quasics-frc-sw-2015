// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"
#include "LoggingUtils.h"

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// DEFINE this if we want to log the state reported by the ball sensor (limit
// switch or beam break sensor).
#undef LOG_BALL_SENSOR_STATE

// Conditional compilation flags end here.
///////////////////////////////////////////////////////////////////////////////

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
#if defined(LOG_BALL_SENSOR_STATE)
  static double oldValue = -1;
  double sensorValue = -1;

#if defined(INTAKE_USES_LIMIT_SWITCH)
  static const std::string sensorName("Limit switch");
  sensorValue = conveyorLimitSwitch->Get();
#elif defined(INTAKE_USES_BEAM_SENSOR)
  static const std::string sensorName("Beam sensor");
  sensorValue = conveyorBeamSensor->Get();
#endif

  if (sensorValue != oldValue) {
    // Report status only when it actually changes (to keep us from flooding the
    // console with output).
    std::cout << sensorName << " is " << (sensorValue ? "" : "not ") << "open ("
              << sensorValue << ")" << std::endl;
    oldValue = sensorValue;
  }
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

void Intake::SetBallPickupSpeed(double percent) {
  // Cap the value of percent to -1.0 to +1.0.
  double useSpeed = std::max(-1.0, std::min(1.0, percent));
  intakeMotor.Set(useSpeed);
}

void Intake::SetConveyorSpeed(double percent) {
  // Cap the value of percent to -1.0 to +1.0.
  double useSpeed = std::max(-1.0, std::min(1.0, percent));
  conveyorMotor.Set(useSpeed);
}
