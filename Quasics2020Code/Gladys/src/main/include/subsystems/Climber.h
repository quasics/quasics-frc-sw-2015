/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

/**
 * Climber subsystem for endgame scoring
 */ 
class Climber : public frc2::SubsystemBase {
 public:
  Climber();

/**
 * Move the climber up
 */ 
  void MoveClimberUp();
/**
 * Move the climber down
 */ 
  void MoveClimberDown();
/**
 * Stop the climber
 */ 
  void StopClimber();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX VictorSPX;

  VictorSPX RightClimber;
  VictorSPX LeftClimber;
};
