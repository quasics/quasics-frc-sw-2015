// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerPlate extends SubsystemBase {
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
