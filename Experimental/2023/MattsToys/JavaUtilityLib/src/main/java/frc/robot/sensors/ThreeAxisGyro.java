// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public abstract class ThreeAxisGyro {
  /** Returns a Gyro object that can track rotation on the X axis (roll). */
  public abstract Gyro getRollGyro();

  /** Returns a Gyro object that can track rotation on the Y axis (pitch). */
  public abstract Gyro getPitchGyro();

  /** Returns a Gyro object that can track rotation on the Z axis (yaw). */
  public abstract Gyro getYawGyro();
}
