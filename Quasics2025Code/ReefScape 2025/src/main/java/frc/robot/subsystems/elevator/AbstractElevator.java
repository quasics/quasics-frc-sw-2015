// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractElevator extends SubsystemBase {
  /** Creates a new AbstractElevator. */
  public AbstractElevator() {
  }

  public abstract void setSpeed(double percentSpeed);

  // CODE_REVIEW: This isn't being used, so it should probably be removed.
  public abstract void setVoltage(double voltage);

  public abstract void stop();

  public abstract void resetEncoders();

  public abstract double getPosition();

  public abstract double getVelocity();

  // CODE_REVIEW: Try to avoid using hardware-specific types in your API. A better
  // approach would be to use a more generic type (e.g., a PIDController
  // interface) or to have the abstract class provide a higher-level function
  // (e.g., "setTargetPosition()"), and then have the concrete implementations use
  // the hardware-specific types. This will make it easier to swap out the
  // hardware-specific types in the future, or to implement the same functionality
  // on other hardware (e.g., under simulation).
  public abstract SparkClosedLoopController getPIDController();

  @Override
  public void periodic() {
    // Always invoke the base class version, unless you *know* why you shouldn't do
    // so.
    super.periodic();

    SmartDashboard.putNumber("elevator encoder position", getPosition());
    SmartDashboard.putNumber("elevator encoder velocity", getVelocity());
  }
}
