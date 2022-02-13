// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /** Indicates the current operation the climber is performing. */
  enum class Movement {
    eUp,      ///< Climber is currently extending
    eDown,    ///< Climber is currently retracting
    eStopped  ///< Climber is currently stopped
  };

  void StartExtending();

  void StartRetracting();

  void Stop();

  void EnableBraking(bool value);

  /** Returns the climber's current status (operation). */
  Movement GetCurrentStatus();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  Movement m_currentStatus;
  rev::CANSparkMax m_climberLeft{MotorIds::SparkMax::LEFT_CLIMBER_MOTOR_ID,
                                 rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climberRight{MotorIds::SparkMax::RIGHT_CLIMBER_MOTOR_ID,
                                  rev::CANSparkMax::MotorType::kBrushless};

  std::unique_ptr<frc::MotorControllerGroup> m_climbers;
};
