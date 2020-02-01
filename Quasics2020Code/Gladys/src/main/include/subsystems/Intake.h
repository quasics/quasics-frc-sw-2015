/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  // Controlling the motor that actually sucks in balls from the floor.
  void TurnSuctionOn();
  void TurnSuctionOff();

  //Controlling the motor that shoots the balls.

  void TurnShooterOn();
  void TurnShooterOff();

  // Controlling the motor at the shoulder joint.

  void RotateShoulderUp();
  void RotateShoulderDown();
  void TurnShoulderOff();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax BallIntake;
  rev::CANSparkMax IntakeArm;
  rev::CANSparkMax BallShooter;
};
