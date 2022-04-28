// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class RearRoller : public frc2::SubsystemBase {
 public:
  RearRoller();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void SetRollerSpeed(double speed);

  void Stop() {
    SetRollerSpeed(0);
  };

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_rearRoller{MotorIds::SparkMax::REAR_ROLLER_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
};
