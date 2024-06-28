// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// DEFINE this if we're using a limit switch for ball detection in the chamber.
#undef INTAKE_USES_LIMIT_SWITCH
// DEFINE this if we're using a beam sensor for ball detection in the chamber.
#define INTAKE_USES_BEAM_SENSOR

// Conditional compilation flags end here.
///////////////////////////////////////////////////////////////////////////////

/**
 * Intake subsystem, comprising the actual "floor pickup" control, and the
 * conveyor system used to move balls from the hopper to the shooter.
 */
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Controls what the commmands that intake (or exhaust) the balls.
  void IntakeBallOn();
  void IntakeBallReverse();   // CODE_REVIEW: Defined, but not implemented?
  void IntakeBallOff();

  void ConveyBallOn();
  void ConveyBallOnSlow();
  void ConveyBallReverse();
  void ConveyBallOff();

  void OnlyIntakeOn();
  void OnlyIntakeReverse();
  void OnlyIntakeOff();

  bool IsBallInChamber();

 public:
  /**
   * Sets the speed for the ball pick-up mechanism.
   * @param percent
   *           a value from 1.0 (full forward) to -1.0 (full reverse); 0 ==
   *           stop.
   */
  void SetBallPickupSpeed(double percent);
  /**
   * Sets the speed for the conveyor used to transfer balls from the
   * "hopper" to the shooter.
   *
   * @param percent
   *           a value from 1.0 (full forward) to -1.0 (full reverse); 0 ==
   *           stop.
   */
  void SetConveyorSpeed(double percent);

 public:
  static constexpr double MOTOR_OFF_POWER = 0;
  static constexpr double MOTOR_SLOW_POWER = 0.25;
  static constexpr double MOTOR_FAST_POWER = 0.75;
  static constexpr double MOTOR_FULL_POWER = 1.0;

 private:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX intakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX conveyorMotor;

#if defined(INTAKE_USES_LIMIT_SWITCH)
  std::shared_ptr<frc::DigitalInput> conveyorLimitSwitch;
#elif defined(INTAKE_USES_BEAM_SENSOR)
  std::shared_ptr<frc::DigitalInput> conveyorBeamSensor;
#endif
};
