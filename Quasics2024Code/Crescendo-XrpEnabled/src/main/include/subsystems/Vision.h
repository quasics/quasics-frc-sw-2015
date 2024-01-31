// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  bool AprilTagTargetFound(int TargetID);

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photon::PhotonCamera camera{"USB_Camera"};
};
