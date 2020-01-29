/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();
  
  void Stop ();
  void SetPower(double rightPower, double leftPower);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax leftFront;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax leftRear; 
  rev::CANSparkMax rightRear; 
  
};
