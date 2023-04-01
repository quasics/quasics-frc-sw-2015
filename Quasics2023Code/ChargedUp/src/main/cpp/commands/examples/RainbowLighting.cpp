// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/examples/RainbowLighting.h"

#include <frc/util/Color.h>

/** Maximum legal value in WPILib for hues. */
constexpr int MAX_HUE = 180;

RainbowLighting::RainbowLighting(Lighting* lighting,
                                 units::second_t delayBeforeAdvancing,
                                 int extraGapBetweenColors)
    : m_lighting(lighting),
      m_delayBeforeAdvancing(std::max(0_s, delayBeforeAdvancing)),
      m_extraGapBetweenColors(std::max(0, m_extraGapBetweenColors)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);

  m_colorFunction = [this](int pos) {
    // Compute the position on the color wheel for the current pixel.
    int effectivePosition =
        (pos + m_offset + m_extraGapBetweenColors) % MAX_HUE;

    // Compute the RGB values for the color, based on the color wheel position.
    const auto c = frc::Color::FromHSV(
        // Hue: position on the "color wheel", ranging from red @ 0 to green @
        // 60, to blue @ 120, and back to red
        effectivePosition,
        // Saturation: "depth of color" or "colorfulness", relative to its own
        // brightness (lower values shift toward white)
        255,
        // Value: "brightness" (0-255, with 255 being "full brightness")
        255);

    // Return the LEDData (converting RGB data from [0.0-1.0] values to 0-255).
    return frc::AddressableLED::LEDData{int(c.red * 255), int(c.green * 255),
                                        int(c.blue * 255)};
  };
}

void RainbowLighting::Initialize() {
  m_offset = 0;
  m_timer.Restart();
  m_lighting->SetLightColors(m_colorFunction);
}

void RainbowLighting::Execute() {
  // See if any requested delay has elapsed before updating "offset" to advance
  // the colors along the strip.
  if (m_delayBeforeAdvancing.value() == 0 ||
      m_timer.HasElapsed(m_delayBeforeAdvancing)) {
    m_timer.Reset();
    ++m_offset;
  }

  // Update the colors of the LEDs on the strip.
  m_lighting->SetLightColors(m_colorFunction);
}

void RainbowLighting::End(bool interrupted) {
  m_lighting->SetDefaultLighting();
}

bool RainbowLighting::IsFinished() {
  return false;
}
