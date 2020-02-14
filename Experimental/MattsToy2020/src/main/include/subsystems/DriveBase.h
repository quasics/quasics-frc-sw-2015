/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <functional>

class DriveBase : public frc2::SubsystemBase {
 public:
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetMotorPower(double leftPower, double rightPower);
  void Stop() {
    SetMotorPower(0, 0);
  }

  void EnableTurboMode() {
    powerAdjuster = turboPowerAdjuster;
  }
  void DisableTurboMode() {
    powerAdjuster = standardPowerAdjuster;
  }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

  static const std::function<double(double)> joystickRangeLimiter;
  static const std::function<double(double)> standardPowerAdjuster;
  static const std::function<double(double)> turboPowerAdjuster;

  std::function<double(double)> powerAdjuster = turboPowerAdjuster;
};
