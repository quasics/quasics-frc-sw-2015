// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  /** Maximum linear speed is 3 meters per second. */
  public static final double MAX_SPEED = 3.0;
  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final double MAX_ANGULAR_SPEED = Math.PI;

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase() {
  }

  protected abstract DifferentialDriveOdometry getOdometry();

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  /**
   * 
   * @param xSpeed the speed for the x axis (in m/s)
   * @param rot    the rotation (in radians/s)
   */
  public abstract void arcadeDrive(double xSpeed, double rot);
}
