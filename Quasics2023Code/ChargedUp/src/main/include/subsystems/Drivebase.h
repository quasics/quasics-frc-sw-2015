// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>

#include <units/length.h>

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  //values are in percentages from 1 to -1
  void TankDrive(double leftPower, double rightPower);

  void Stop();

  units::meter_t GetLeftDistance();

  units::meter_t GetRightDistance();

  void ResetEncoders();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  int x;
  std::string * y;

  std::unique_ptr<frc::DifferentialDrive> m_drive;

};
