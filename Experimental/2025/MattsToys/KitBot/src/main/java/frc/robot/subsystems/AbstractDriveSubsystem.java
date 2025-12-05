// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractDriveSubsystem extends SubsystemBase {
  /** Creates a new AbstractDriveSubsystem. */
  public AbstractDriveSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets the speed of the drive motors
  public abstract void driveArcade(double xSpeed, double zRotation);
}
