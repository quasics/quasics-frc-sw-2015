// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class FloorEjection : public frc2::SubsystemBase {
 public:
  FloorEjection();

  void SetFloorEjectionPower(double power);

  void Stop();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::VictorSPX m_floorEjectionMotor{
      MotorIds::VictorSPX::GAME_PIECE_FLIPPER_ID};
};
