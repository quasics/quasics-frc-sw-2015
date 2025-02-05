// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbstractDrivebase extends SubsystemBase {
  /** Creates a new AbstractDrivebase. */

  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);

  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  public AbstractDrivebase() {

  }

  public final void stop() {

  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds){
  
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
