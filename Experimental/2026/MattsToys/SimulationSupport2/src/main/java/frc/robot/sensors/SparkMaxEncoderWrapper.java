// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Convenience wrapper, allowing a CANSparkMax's RelativeEncoder to be used as a
 * normal WPLib Encoder.
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
  public void reset() {
    encoder.setPosition(0);
  }

  @Override
  public Distance getPosition() {
    return Meters.of(encoder.getPosition());
  }

  @Override
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(encoder.getVelocity());
  }

  @Override
  public void close() throws IOException {
    // No-op: SparkMax encoder doesn't expose this method.
  }
}
