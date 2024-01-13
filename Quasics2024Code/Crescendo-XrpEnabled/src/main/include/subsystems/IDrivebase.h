// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"

class IDrivebase : public frc2::SubsystemBase {
 public:
  IDrivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};

// this is a test
