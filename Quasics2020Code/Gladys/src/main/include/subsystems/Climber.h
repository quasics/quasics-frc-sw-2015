/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  void MoveClimberUp(); //move up

  void MoveClimberDown(); //move down

  void StopClimber(); //stop moving (brake)

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX VictorSPX;

  VictorSPX RightClimber;
  VictorSPX LeftClimber;
};
