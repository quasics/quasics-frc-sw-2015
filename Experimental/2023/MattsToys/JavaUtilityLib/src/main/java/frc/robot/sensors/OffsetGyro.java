// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Wrapper around a Gyro, allowing us to reset it "locally", without affecting the original gyro's
 * data.
 */
public class OffsetGyro implements Gyro {
  /** The Gyro used to obtain actual readings */
  private final Gyro m_sourceGyro;

  /** Used by "reset()" to establish a new baseline for the angle. */
  private double m_calibrationOffset = 0;

  /**
   * Constructs an OffsetGyro, wrapped around a source Gyro.
   *
   * @param sourceGyro the Gyro used to obtain actual readings
   */
  public OffsetGyro(final Gyro sourceGyro) {
    m_sourceGyro = sourceGyro;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void calibrate() {}

  @Override
  public void reset() {
    m_calibrationOffset = m_sourceGyro.getAngle();
  }

  @Override
  public double getAngle() {
    return m_sourceGyro.getAngle() - m_calibrationOffset;
  }

  @Override
  public double getRate() {
    return m_sourceGyro.getRate();
  }
}
