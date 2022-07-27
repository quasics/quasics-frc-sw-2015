// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Lighting.h>

#include "utils/LightingShifter.h"

/**
 * An example of how to use the "LightingShifter" utility class with a simple
 * function to build a fixed pattern for the lights, in order to have a command
 * that will make the pattern "crawl" along the strips.
 */
class SimpleLightingCrawler
    : public frc2::CommandHelper<frc2::CommandBase, SimpleLightingCrawler> {
 public:
  /// Constructor.  Sets up the "shifter" for use when the command is running.
  /// @param lighting  the lighting subsystem
  SimpleLightingCrawler(Lighting* lighting)
      : m_shifter(lighting,  // lighting subsystem
                  20.0_s,  // total time it should take to make one full circuit
                           // of the strip
                  true     // the pattern should crawl "down" the strip
        ) {
    AddRequirements(lighting);
  }

  void Initialize() override {
    // Start the shifter running.
    m_shifter.Start(m_colors);
  }

  void Execute() override {
    // Tell the shifter to update the pattern (if appropriate).
    m_shifter.Update(m_colors);
  }

  void End(bool interrupted) override {
    // Tell the shifter to stop its timer (and clean up the lights).
    m_shifter.Stop();
  }

 private:
  LightingShifter m_shifter;

  // This is a simple function to define a pattern for the lights that is a
  // repeating sequence of "5 off, 5 green".
  const Lighting::FrcColorFunction m_colors = [](int pos) {
    constexpr auto cellCount = 5;
    constexpr auto groupSize = cellCount * 2;
    const auto posInGroup = pos % groupSize;
    return frc::Color(posInGroup < cellCount ? frc::Color::kBlack
                                             : frc::Color::kGreen);
  };
};
