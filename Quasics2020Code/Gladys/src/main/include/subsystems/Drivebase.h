/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <iostream>

#include "Constants.h"

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void SetMotorPower(double leftPower, double rightPower);
  void Stop() {
    SetMotorPower(0, 0);
  }
  void EnableTurboMode() {
    std::cout << "Switching to TURBO" << std::endl;
    powerScaling = DriveBaseConstants::TurboMaxPower;
  }
  void DisableTurboMode() {
    std::cout << "Switching to normal" << std::endl;
    powerScaling = DriveBaseConstants::StandardMaxPower;
  }
  void SwitchFace() {
    frontIsForward = !frontIsForward;
    std::cout << "Switching to front being "
              << (frontIsForward ? "forward" : "backward") << std::endl;
  }

  void DisplayEncoderValues();

  double GetLeftFrontEncoderPosition();

  double GetRightFrontEncoderPosition();

  void ResetEncoderPositions();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;
  rev::CANEncoder leftFrontEncoder =
      leftFront.GetEncoder();  //.GetPosition and .GetVelocity
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();

  double powerScaling = DriveBaseConstants::StandardMaxPower;
  bool frontIsForward = true;
};
