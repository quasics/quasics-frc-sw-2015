/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class SwissArmySubsystem : public frc2::SubsystemBase {
 public:
  SwissArmySubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void ClimbUp() {
    SetElevatorPower(ElevatorConstants::kRisingPower,
                     ElevatorConstants::kRisingPower);
  }
  void ClimbDown() {
    SetElevatorPower(-ElevatorConstants::kLoweringPower,
                     -ElevatorConstants::kLoweringPower);
  }
  void StopClimbing() {
    SetElevatorPower(0, 0);
  }

  void RotateShoulderDown();
  void RotateShoulderUp();
  void StopMovingShoulder();

 private:
  // Positive values == lifting; negative values == lowering
  void SetElevatorPower(double left, double right);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Convenience definition (shortening the name).
  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX WPI_VictorSPX;

  // Example: shoulder joint on an example subsystem.
  WPI_VictorSPX shoulder{CANBusConstants::VictorSpxIds::ShoulderJointId};

  // Example: elevator motors on an example subsystem.
  WPI_VictorSPX rightElevatorMotor{
      CANBusConstants::VictorSpxIds::RightElevatorMotorId};
  WPI_VictorSPX leftElevatorMotor{
      CANBusConstants::VictorSpxIds::LeftElevatorMotorId};
};
