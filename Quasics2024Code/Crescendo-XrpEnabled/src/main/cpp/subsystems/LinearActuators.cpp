// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LinearActuators.h"

#include <units/time.h>

/**
 * Configures the bounds for an AndyMark L16 linear actuator, per their docs.
 *
 * @see https://www.andymark.com/products/actuator-l16-r-140mm-stroke-35-1-6v
 * @see http://tinyurl.com/45t2ffyc
 * @see https://www.chiefdelphi.com/t/anyone-have-servo-example-code/155951/9
 */
void LinearActuators::ConfigureAndyMarkL16(frc::Servo& servo) {
  // Configure the boundaries for this device, which are different from a
  // bog-standard servo.
  servo.SetBounds(2.0_ms,  // max (per docs)
                  1.8_ms,  // deadbandMax (per docs)
                  1.5_ms,  // center (per docs)
                  1.2_ms,  // deadbandMin (per docs)
                  1.0_ms   // min (per docs)
  );
}

LinearActuators::LinearActuators() {
  SetName("LinearActuators");
  ConfigureAndyMarkL16(m_leftPositionServo);
  ConfigureAndyMarkL16(m_rightPositionServo);
}

void LinearActuators::ExtendLinearActuators() {
  m_leftPositionServo.SetSpeed(+1.00);
  m_rightPositionServo.SetSpeed(+1.00);
}

void LinearActuators::RetractLinearActuators() {
  m_leftPositionServo.SetSpeed(-1.00);
  m_rightPositionServo.SetSpeed(-1.00);
}
