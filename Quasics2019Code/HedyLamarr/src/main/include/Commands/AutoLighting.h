/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class AutoLighting : public frc::Command {
 public:
  AutoLighting();
  void Initialize();
  void Execute() override;
  bool IsFinished() override;
  private:
  std::string lastModeCommand = "";
  std::string lastColorCommand = "";
  void transmitColor(std::string colorCommand, bool alwaysSend = false);
  void transmitMode(std::string modeCommand, bool alwaysSend = false);
};
