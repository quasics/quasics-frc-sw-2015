// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.io.IOException;

/**
 * Convenience wrapper, allowing a TalonFX to be read in the same (general)
 * way as a normal WPLib Encoder.
 */
public class TalonEncoderWrapper implements TrivialEncoder {
  final TalonFX m_motorController;
  final Distance m_outerCircumference;

  /**
   * Constructor.
   *
   * @param motorController the TalonFX motor controller whose encoder we want to
   *                        wrap
   * @param outerDiameter   the outer diameter of the wheel being turned by the
   *                        motor (used to convert "revolutions" to linear
   *                        distance)
   */
  public TalonEncoderWrapper(TalonFX motorController, Distance outerDiameter) {
    m_motorController = motorController;
    m_outerCircumference = outerDiameter.times(Math.PI);
  }

  public void close() throws IOException {
    // No-op: close the motor controller, which will also close the encoder.
  }

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
}
