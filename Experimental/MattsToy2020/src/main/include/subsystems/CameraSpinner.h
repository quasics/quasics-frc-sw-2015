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

/**
 * Sample subsystem, controlling a single servo used to orient a camera.
 *
 * Along with allowing the camera to be turned to point ahead of/behind the
 * robot, this example will also allow the current direction to be reported to
 * clients.
 */
class CameraSpinner : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  CameraSpinner();

  /** Used to trigger orienting the camera to the front of the robot. */
  void TurnToFront();

  /** Used to trigger orienting the camera to the rear of the robot. */
  void TurnToRear();

  /**
   * Reportable positions for the camera.  (If the servo's current position is
   * sufficiently close to fully rotated to one direction or the other, we'll
   * report that as the result; otherwise, we'll report the "Inbetween" status.)
   *
   * @see #GetPosition
   */
  enum class Position { Front, Rear, Inbetween };

  /**
   * Returns the (rough) orientation of the camera.
   *
   * If the servo's current position is sufficiently close to fully rotated to
   * one direction or the other, we'll report that as the result; otherwise,
   * we'll report the "Inbetween" status.)
   */
  Position GetPosition();

  /// Will be called periodically whenever the CommandScheduler runs.
  void Periodic() {
  }

 private:
  /// Servo position used when pointing camera to the front
  static const double kFrontValue;
  /// Servo position used when pointing camera to the rear
  static const double kRearValue;
  /// Tolerance value used when reporting from GetPosition() method.
  static const double kPositionTolerance;

  /// Servo used to orient the camera.
  frc::Servo cameraRotator;
};
