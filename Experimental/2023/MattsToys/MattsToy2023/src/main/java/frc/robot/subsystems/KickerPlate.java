// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Trivial class for the "kicker" on the 2023 robot, which is used to dump game pieces out of the
 * intake storage area.
 */
public class KickerPlate extends SubsystemBase {
  // Note that this assumes we're using it as a duty cycle (absolute) encoder. If
  // we're using it as a quadrature (relative) encoder, then we'd need to use 2
  // DIO ports on the Rio, and use an Encoder object.
  //
  // @see
  // https://www.chiefdelphi.com/t/rev-through-bore-encoder-programming/424784
  // @see https://docs.revrobotics.com/through-bore-encoder/application-examples
  private DutyCycleEncoder m_rotaryEncoder =
      new DutyCycleEncoder(new DigitalInput(Constants.ROTARY_ENCODER_CHANNEL));

  /** Creates a new KickerPlate. */
  public KickerPlate() {}

  @Override
  public void periodic() {
    if (Constants.DebugSettings.KickerPlateDebugEnabled) {
      System.err.println(
          "Rotary Encoder: connected="
              + m_rotaryEncoder.isConnected()
              + ", position="
              + m_rotaryEncoder.getAbsolutePosition());
    }
  }
}
