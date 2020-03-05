/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

/// @todo (Nurfadil) Document this class (using JavaDoc format).
class Exhaust : public frc2::SubsystemBase {
 public:
  Exhaust();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  // Lower motor, only used to move the ball into the primary holding area.
  void PushBallUp();
  void PushBallDown();
  void PushBallOff();

  void ShootBallOn();
  void ShootBallDown();

  void ShootBallOff();

 private:
  /// Convenient alias for the underlying motor type.
  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX WPI_VictorSPX;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  /// @todo (Nurfadil) Either rename the motor control data members so that they
  // more clearly indicate what they do/where they're located, or add comments
  // providing this information.  (Better still, do both!)

  WPI_VictorSPX Shoot;
  WPI_VictorSPX Push;
};
