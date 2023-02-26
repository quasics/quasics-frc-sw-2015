// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Provides access to discrete Gyros for each of the axes supported by the Pigeon2. */
public class PigeonThreeAxisGyroWrapper implements ThreeAxisGyro {
  /** The wrapped Pigeon2. */
  private Pigeon2 m_pigeon;

  /**
   * Constructor.
   *
   * @param deviceNumber CAN ID for the Pigeon2 being wrapped.
   */
  public PigeonThreeAxisGyroWrapper(int deviceNumber) {
    m_pigeon = new Pigeon2(deviceNumber);
  }

  /**
   * Constructor.
   *
   * @param deviceNumber CAN ID for the Pigeon2 being wrapped.
   * @param canbus Name of the CANbus on which the Pigeon2 is wired.
   */
  public PigeonThreeAxisGyroWrapper(int deviceNumber, String canbus) {
    m_pigeon = new Pigeon2(deviceNumber, canbus);
  }

  @Override
  public Gyro getRollGyro() {
    return new SingleAxisWrapper(m_pigeon::getRoll);
  }

  @Override
  public Gyro getPitchGyro() {
    return new SingleAxisWrapper(m_pigeon::getPitch);
  }

  @Override
  public Gyro getYawGyro() {
    return new SingleAxisWrapper(m_pigeon::getYaw);
  }
}
