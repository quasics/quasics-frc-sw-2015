// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RainbowLighting.h"

RainbowLighting::RainbowLighting(Lighting* lighting,
                                 units::second_t secondsBeforeAdvancing,
                                 int extraGapBetweenColors)
    : m_lighting(lighting), m_secondsBeforeAdvancing(secondsBeforeAdvancing) {
  AddRequirements(m_lighting);

  extraGapBetweenColors = std::max(0, extraGapBetweenColors);

  m_colorFunction = [this, extraGapBetweenColors](int pos) {
    int effectivePosition =
        (pos + this->m_offset + extraGapBetweenColors) % MAX_HUE;
    // h - the h value [0-180] - ranges from red @ 0 to green @ 60, to blue
    // @ 120, and back to red s - the s value [0-255] - "depth of color"
    // (lower values shift toward white) (colorfulness, relative to its own
    // brightness) v - the v value [0-255] - brightness
    frc::AddressableLED::LEDData result;
    result.SetHSV(effectivePosition, 255, 255);
    return result;
  };
}

// Called when the command is initially scheduled.
void RainbowLighting::Initialize() {
  // Reset the values used to control color advance.
  m_offset = 0;
  m_timer.Reset();
  m_timer.Start();

  // Update the colors of the LEDs on the strip.
  m_lighting->SetStripColors(m_colorFunction);
}

// Called repeatedly when this Command is scheduled to run
void RainbowLighting::Execute() {
  // See if any requested delay has elapsed before updating "offset" to advance
  // the colors along the strip.
  if (m_secondsBeforeAdvancing.value() <= 0 ||
      m_timer.HasElapsed(m_secondsBeforeAdvancing)) {
    m_timer.Reset();
    ++m_offset;
  }

  // Update the colors of the LEDs on the strip.
  m_lighting->SetStripColors(m_colorFunction);
}
