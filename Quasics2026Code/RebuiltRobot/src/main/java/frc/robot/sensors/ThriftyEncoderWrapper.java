// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.util.Conversion;
import com.thethriftybot.util.Conversion.PositionUnit;
import com.thethriftybot.util.Conversion.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.io.IOException;

/**
 * Convenience wrapper, allowing a ThriftyNova to be read in the same (general)
 * way as a normal WPLib Encoder.
 */
public class ThriftyEncoderWrapper implements TrivialEncoder {
  /** Wrapped Thrifty Nova controller, providing access to encoder data. */
  final ThriftyNova m_motorController;

  /**
   * Distance travelled per turn of the outer wheel. (Used to convert
   * "revolutions" to linear distance.)
   */
  final Distance m_rotationDistance;

  /**
   * Thrifty conversion object, used to translate native velocity units to RPMs.
   */
  final Conversion m_shooterConverter =
      new Conversion(VelocityUnit.ROTATIONS_PER_MIN, EncoderType.INTERNAL);

  /**
   * Thrifty conversion object, used to translate native positional units to
   * rotations.
   */
  final Conversion m_distanceConverter =
      new Conversion(PositionUnit.ROTATIONS, EncoderType.INTERNAL);

  /**
   * Constructor.
   *
   * @param motorController    ThriftyNova object being wrapped for "normal" use
   * @param wheelOuterDiameter outer diameter of the wheel being turned by the
   *                           motor
   */
  public ThriftyEncoderWrapper(
      ThriftyNova motorController, Distance wheelOuterDiameter) {
    m_rotationDistance = wheelOuterDiameter.times(Math.PI);
    m_motorController = motorController;
  }

  @Override
  public Distance getPosition() {
    final double currentRevolutions =
        m_distanceConverter.fromMotor(m_motorController.getPosition());
    return m_rotationDistance.times(currentRevolutions);
  }

  @Override
  public LinearVelocity getVelocity() {
    final double currentRPM =
        m_shooterConverter.fromMotor(m_motorController.getVelocity());
    final double revsPerSec = currentRPM * 60;
    return MetersPerSecond.of(m_rotationDistance.in(Meters) *revsPerSec);
  }

  @Override
  public void reset() {
    m_motorController.setEncoderPosition(0);
  }

  @Override
  public void close() throws IOException {
    // No-op: ThriftyNova should be closed through the MotorController
    // interface.
  }
}
