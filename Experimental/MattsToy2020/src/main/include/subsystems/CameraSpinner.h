/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include "utils/ComparisonHelpers.h"

class CameraSpinner : public frc2::SubsystemBase {
 public:
  CameraSpinner();

  void TurnToFront();
  void TurnToRear();

  enum class Position { Front, Rear, Inbetween };
  Position GetPosition();

  /// Will be called periodically whenever the CommandScheduler runs.
  void Periodic() {
  }

 private:
  static const double kFrontValue;
  static const double kRearValue;
  static const double kPositionTolerance;

  frc::Servo cameraRotator;
};
