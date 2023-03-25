// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ConfigSettings.h"
#include "subsystems/Lighting.h"

/**
 * Houses all of the functions the lighting system will be able to do during
 * a match. Includes alliance color, human player signals, and drive base
 * data.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MatchPlayLighting
    : public frc2::CommandHelper<frc2::CommandBase, MatchPlayLighting> {
 public:
  MatchPlayLighting(Lighting* lighting, ConfigSettings* configSettings);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc::AddressableLED::LEDData GetComplexColorFunction(
      frc::AddressableLED::LEDData allianceColor,
      RequestedPayload requestedPayload, int pos);

 private:
  static frc::AddressableLED::LEDData SimpleColorFunction(int position);

  // Data members
 private:
  Lighting* m_lighting;
  ConfigSettings* m_configSettings;
};
