/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <arpa/inet.h>
#include <frc/commands/Subsystem.h>

class LightingSubsystem : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  int udpSocket;
  sockaddr_in broadcastAddress;

 public:
  LightingSubsystem();
  virtual ~LightingSubsystem();
  void InitDefaultCommand() override;

  bool sendCommandToArduino(std::string cmd);
};
