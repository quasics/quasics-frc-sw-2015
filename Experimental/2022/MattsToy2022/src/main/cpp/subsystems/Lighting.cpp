// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

const frc::AddressableLED::LEDData BLACK{0, 0, 0};
const frc::AddressableLED::LEDData WHITE{255, 255, 255};

Lighting::Lighting() {
  // Start with lights alternating between black (off) and white.
  std::generate(m_ledBuffer.begin(), m_ledBuffer.end(),
                // "Lambda" function, used to flip every other light on.
                [] {
                  // Each time the function is called, this will remember what
                  // happened *last* time.  (And the first time it's called, it
                  // will be equal to "true".)
                  static bool turnOn = true;

                  // Decide what we will do this time (the opposite of *last*
                  // time).
                  turnOn = !turnOn;

                  // Return the color for the next cell in the strip.
                  return turnOn ? WHITE : BLACK;
                });

  // Note: Length is expensive to set, so only set it once, then just update
  // the buffer and send it in as new data for the strip.
  m_ledStrip.SetLength(m_ledBuffer.size());
  m_ledStrip.SetData(m_ledBuffer);
  m_ledStrip.Start();
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
