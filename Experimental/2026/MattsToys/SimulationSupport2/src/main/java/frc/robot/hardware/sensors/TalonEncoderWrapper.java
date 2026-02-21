// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.sensors;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.io.IOException;

/**
 * Convenience wrapper, allowing a TalonFX to be read in the same (general)
 * way as a normal WPLib Encoder.
 *
 * Note: Phoenix 6 moved away from "native units" (ticks) coming back from the
 * encoder, and uses "rotations" as the standard unit for everything it does.
 * CTRE's philosophy now is that the scaling should happen in your Java code,
 * with the intention that this will prevent "magic numbers" from being hidden
 * inside the motor controller's memory/configuration, making your code much
 * easier to debug.
 *
 * However, this means that the TalonFX's "position" and "velocity" values are
 * now in "rotations" and "rotations per second", respectively, and thus need to
 * be converted to linear distance and velocity using the wheel circumference,
 * which this wrapper will do.
 */
public class TalonEncoderWrapper implements TrivialEncoder {
  /** Wrapped TalonFX motor controller, providing access to encoder data. */
  final TalonFX m_motorController;

  /**
   * Distance travelled per turn of the outer wheel. (Used to convert
   * "revolutions" to linear distance, etc.)
   */
  final Distance m_outerCircumference;

  /**
   * Constructor.
   *
   * @param motorController the TalonFX motor controller whose encoder we want
   *     to
   *                        wrap
   * @param outerDiameter   the outer diameter of the wheel being turned by the
   *                        motor (used to convert "revolutions" to linear
   *                        distance)
   */
  public TalonEncoderWrapper(TalonFX motorController, Distance outerDiameter) {
    m_motorController = motorController;
    m_outerCircumference = outerDiameter.times(Math.PI);
  }

  //
  // TrivialEncoder implementation
  //

  @Override
  public Distance getPosition() {
    double rotations = m_motorController.getPosition().getValueAsDouble();
    return m_outerCircumference.times(rotations);
  }

  @Override
  public LinearVelocity getVelocity() {
    double rps = m_motorController.getVelocity().getValueAsDouble();
    return m_outerCircumference.times(rps).div(Seconds.of(1));
  }

  @Override
  public void reset() {
    m_motorController.setPosition(0);
  }

  //
  // Closeable implementation
  //

  @Override
  public void close() throws IOException {
    // No-op: close the motor controller, which will also close the encoder.
  }
}
