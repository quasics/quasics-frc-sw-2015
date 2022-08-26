// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * Command to set the robot's lights to a given color (when activated).
 */
class SetLightsToColor
    : public frc2::CommandHelper<frc2::CommandBase, SetLightsToColor> {
 public:
  /**
   * Constructor.
   *
   * @param lighting  pointer to Lighting subsystem
   * @param c         color to which the lights should be set
   */
  SetLightsToColor(Lighting* lighting, Lighting::StockColor c);

  // Standard "Command" functions.
 public:
  void Execute() override;

  // Data members.
 private:
  Lighting* m_lighting;
  const Lighting::StockColor m_color;
};
