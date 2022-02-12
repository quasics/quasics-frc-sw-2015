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

  enum class Movement { eUp, eDown, eStopped };

  void StartExtracting();

  void StartRetracting();

  void Stop();

  void EnableBrakeing(bool value);

  Climber::Movement GetCurrentStatus();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  Movement currentStatus;
  rev::CANSparkMax m_ClimberLeft{MotorIds::SparkMax::LEFT_CLIMBER_MOTOR_ID,
                                 rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_ClimberRight{MotorIds::SparkMax::RIGHT_CLIMBER_MOTOR_ID,
                                  rev::CANSparkMax::MotorType::kBrushless};

  std::unique_ptr<frc::MotorControllerGroup> m_Climbers;
};
