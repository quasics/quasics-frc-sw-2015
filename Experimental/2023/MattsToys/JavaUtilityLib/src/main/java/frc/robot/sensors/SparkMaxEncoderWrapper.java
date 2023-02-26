// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.revrobotics.RelativeEncoder;

/**
 * Convenience wrapper, allowing a CANSparkMax's RelativeEncoder to be used as a normal WPLib
 * Encoder.
 */
public class SparkMaxEncoderWrapper implements TrivialEncoder {
  /** Wrapped Spark Max relative encoder. */
  final RelativeEncoder encoder;

  /**
   * Constructor.
   *
   * @param encoder the relative encoder being wrapped for "normal" use.
   */
  public SparkMaxEncoderWrapper(RelativeEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void reset() {
    encoder.setPosition(0);
  }
}
