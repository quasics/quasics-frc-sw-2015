// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public interface DriveBaseWith3AxisGyroInterface extends DriveBaseInterface {

  /** Returns a Gyro to be used in looking at the robot's roll (rotation on X-axis). */
  public abstract Gyro getRollGyro();

  /** Returns a Gyro to be used in looking at the robot's pitch (rotation on Y-axis). */
  public abstract Gyro getPitchGyro();
}
