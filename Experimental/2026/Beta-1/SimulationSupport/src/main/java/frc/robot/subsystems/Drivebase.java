// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;

public class Drivebase extends SubsystemBase implements IDrivebase {
  /** Creates a new Drivebase. */
  public Drivebase() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void driveArcade(double forward, double rotation) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveArcade'");
  }

  @Override
  public void tankDrive(double leftSpeed, double rightSpeed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'tankDrive'");
  }
}
