// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Wrapper around an IGyro, allowing us to reset it "locally", without affecting
 * the original gyro's data.
 *
 * I foresee this as being useful under at least two different circumstances:
 * <ol>
 * <li>
 * When we're wrapping a multi-axis gyro/ALU inside of the single-axis "IGyro"
 * interface, and want to be able to support resetting data for that axis under
 * some circumstances, without impacting the others.
 * </li>
 * <li>
 * When we're performing some temporally-scoped set of operations (e.g., while a
 * command is running) and want to have an easy reference point (i.e., 0)
 * without affecting the overall use of a gyro/ALU. For example, the gyro might
 * be used on a continuous basis to maintain data for odometry/pose estimation,
 * and thus resetting the actual gyro would "break" that processing. However, we
 * might want to simplify the handling of a command like "turn N degrees" by
 * allowing the code to use 0 as a reference point (and this class would be able
 * to support that).
 * </li>
 * </ol>
 */
public class OffsetGyro implements IGyro {
  /** The Gyro used to obtain actual readings */
  private final IGyro m_sourceGyro;

  /** Used by "reset()" to establish a new baseline for the angle. */
  private Angle m_calibrationOffset = Degrees.of(0);

  /**
   * Constructs an OffsetGyro, wrapped around a source Gyro.
   *
   * @param sourceGyro the Gyro used to obtain actual readings
   */
  public OffsetGyro(final IGyro sourceGyro) {
    m_sourceGyro = sourceGyro;
  }

  @Override
  public void calibrate() {
    System.out.println("Note: calibrating offset gyro");
    reset();
  }

  @Override
  public void reset() {
    m_calibrationOffset = m_sourceGyro.getAngle();
  }

  @Override
  public Angle getAngle() {
    return m_sourceGyro.getAngle().minus(m_calibrationOffset);
  }

  @Override
  public AngularVelocity getRate() {
    return m_sourceGyro.getRate();
  }

  @Override
  public Rotation2d getRotation2d() {
    return new Rotation2d((getAngle()));
  }
}
