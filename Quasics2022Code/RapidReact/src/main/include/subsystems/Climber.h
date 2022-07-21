// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "FORCOMPETITIONRobotConstants.h"

class Climber : public frc2::SubsystemBase {
  // Inner types
 public:
  /** Indicates the current operation the climber is performing. */
  enum class Movement {
    eUp,      ///< Climber is currently extending
    eDown,    ///< Climber is currently retracting
    eStopped  ///< Climber is currently stopped
  };

  // Climber implementation.
 public:
  Climber();

  /**
   * Triggers extension of the climbing arms, if they aren't already fully
   * extended.
   */
  void StartExtending();

  /**
   * Triggers retraction of the climbing arms, if they aren't already fully
   * retracted.
   */
  void StartRetracting();

  void ExtendOneClimber(bool isLeft);

  void RetractOneClimber(bool isLeft);
  /**
   * Stops the climbing arms if they are currently extending/retracting.
   *
   * Note: this does not affect "braking mode" for the arms.
   */
  void Stop();

  /**
   * Turns "braking mode" on/off.  (In other words, should the motor controller
   * try to prevent any motion when power has been cut, or should it allow
   * "coasting"/free rotation).
   *
   * @param value if true, enable "breaking mode"; otherwise, allow "coasting"
   */
  void EnableBraking(bool value);

  /**
   * Returns true iff the arms are fully extended (based on sensor readings).
   */
  bool IsFullyExtended();

  /**
   * Returns true iff the arms are fully retracted (based on sensor readings).
   */
  bool IsFullyRetracted();

  /** Returns the climber's current status (operation). */
  Movement GetCurrentStatus();

  // Standard functions for subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Data members.
 private:
  rev::CANSparkMax m_climberLeft{MotorIds::SparkMax::LEFT_CLIMBER_MOTOR_ID,
                                 rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climberRight{MotorIds::SparkMax::RIGHT_CLIMBER_MOTOR_ID,
                                  rev::CANSparkMax::MotorType::kBrushless};
  frc::MotorControllerGroup m_climbers{m_climberLeft, m_climberRight};

  frc::DigitalInput topLimitSwitch{DigitalInput::TOP_LIMIT_SWITCH_ID};

  frc::DigitalInput bottomLimitSwitch{DigitalInput::BOTTON_LIMIT_SWITCH_ID};

  Movement m_currentStatus{Movement::eStopped};
};
