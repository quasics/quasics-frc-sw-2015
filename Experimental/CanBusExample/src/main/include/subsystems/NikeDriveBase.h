/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <rev/CANSparkMax.h>

class NikeDriveBase : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

 public:
  NikeDriveBase();
  void InitDefaultCommand() override;

  void SetMotorPower(double leftSpeed, double rightSpeed);

  void Stop() { SetMotorPower(0.0, 0.0); }

  // The following code is used for testing/example purposes only.
 public:
  enum Motor {
    eLeftFront,
    eLeftRear,
    eRightFront,
    eRightRear,
  };

  void SetSingleMotorPower(Motor m, double power) {
    rev::CANSparkMax* controller = nullptr;
    switch (m) {
      case eLeftFront:
        controller = &leftFront;
      case eLeftRear:
        controller = &leftRear;
      case eRightFront:
        controller = &rightFront;
      case eRightRear:
        controller = &rightRear;
    }
    if (controller != nullptr) {
      controller->Set(power);
    }
  }
};
