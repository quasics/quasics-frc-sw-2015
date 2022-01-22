// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

const frc::AddressableLED::LEDData BLACK{0, 0, 0};
const frc::AddressableLED::LEDData WHITE{255, 255, 255};

Lighting::Lighting() {
  SetName("Lighting");

  // Start with lights alternating between black (off) and white.
  SetStripColors(
      // A simple "lambda" function, which will set any odd-numbered
      // pixels to white, and even-numbered pixels to black.
      [](int pos) {
        if (pos % 2 == 0) {
          return BLACK;
        } else {
          return WHITE;
        }
      });

  // Note: Length is expensive to set, so only set it once, then just update
  // the buffer and send it in as new data for the strip.
  m_ledStrip.SetLength(m_ledBuffer.size());
  m_ledStrip.SetData(m_ledBuffer);
  m_ledStrip.Start();
}

/**
 * Sets the entire strip to the specified color.
 */
void Lighting::SetStripColor(frc::AddressableLED::LEDData color) {
  SetStripColors(
      // We'll use the other function to do the hard work, but we need to pass
      // it a function to call for each pixel, so we'll provide a "lambda"
      // function (something written in-line) to return our fixed color.
      //
      // The "[&]" tells the compiler that any variables from this method that
      // are used in the lambda are automatically "captured by value" (i.e.,
      // just give the lambda its own copy of "color" in this case).
      [=](int) { return color; });
}

/**
 * Sets the color for each LED on the strip, based on the value returned by
 * colorFunction() for each position.
 *
 * @param colorFunction the function to use in generating the color for each LED
 *                      (based on position, running from 0 to "length - 1")
 */
void Lighting::SetStripColors(
    std::function<frc::AddressableLED::LEDData(int pos)> colorFunction) {
  int pos = 0;  // Keeps track of the current pixel # in the lambda below.

  std::generate(
      m_ledBuffer.begin(), m_ledBuffer.end(),
      // A simple lambda to call the color function for each pixel in
      // the strip.
      //
      // The "[&]" tells the compiler that any variables from this method that
      // are used in the lambda are automatically "captured by reference" (i.e.,
      // changes made in the lambda, like incrementing "pos", are actually
      // applied to the original variable in the method, and thus "stick" from
      // one use of the lambda to the next).
      [&] { return colorFunction(pos++); });

  m_ledStrip.SetData(m_ledBuffer);
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
