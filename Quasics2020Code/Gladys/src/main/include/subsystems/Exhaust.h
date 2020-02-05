/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>


class Exhaust : public frc2::SubsystemBase {
 public:
  Exhaust();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void PushBallUp();
  void PushBallDown();
  void ShootBallOn();

  void PushBallOff();
  void ShootBallOff();


 private:

   typedef ctre::phoenix::motorcontrol::can::WPI_TalonSRX WPI_TalonSRX;


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX Shoot;
  WPI_TalonSRX Push; 
};
