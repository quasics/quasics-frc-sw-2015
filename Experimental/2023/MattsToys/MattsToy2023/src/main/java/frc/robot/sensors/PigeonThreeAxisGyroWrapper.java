// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public class PigeonThreeAxisGyroWrapper implements ThreeAxisGyro {
  private Pigeon2 m_pigeon;

  public PigeonThreeAxisGyroWrapper(int deviceNumber) {
    m_pigeon = new Pigeon2(deviceNumber);
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
