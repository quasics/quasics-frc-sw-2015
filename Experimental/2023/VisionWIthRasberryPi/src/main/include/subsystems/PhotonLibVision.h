// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>

class PhotonLibVision : public frc2::SubsystemBase {
 public:
  PhotonLibVision();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void CameraYaw();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //photonlib::PhotonCamera camera{"photonvision"};
  //trying this maybe it will work
  photonlib::PhotonCamera camera{"mmal_service_16.1"};


};
