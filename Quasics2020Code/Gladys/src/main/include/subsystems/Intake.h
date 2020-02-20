/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

// TODO(Nurfadil): Document this class.
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  // Controlling the motor that actually sucks in balls from the floor.
  void TurnSuctionOn();
  void TurnSuctionOnReverse();
  void TurnSuctionOff();

  // Controlling the motor at the shoulder joint.
  void RotateShoulderUp();
  void RotateShoulderDown();
  void TurnShoulderOff();

 private:
  // Convenience definition (shortening the name).
  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX WPI_VictorSPX;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_VictorSPX BallIntake;
  WPI_VictorSPX Shoulder;
};
