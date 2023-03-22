// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

/**
 * TODO: Add comments describing the class as a whole.
 */
// CODE_REVIEW(ethan): This class should have a comment block (above) describing
// what it is/does.
class IntakeRoller : public frc2::SubsystemBase {
 public:
  IntakeRoller();

  void SetRollerSpeed(double percentSpeed);

  void Stop();

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
#ifdef ENABLE_ROLLER_INTAKE_MOTORS
  rev::CANSparkMax m_floorRollerPickupMotor{
      MotorIds::SparkMax::INTAKE_MOTOR_ROLLER_ID,
      rev::CANSparkMax::MotorType::kBrushless};
#endif

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
