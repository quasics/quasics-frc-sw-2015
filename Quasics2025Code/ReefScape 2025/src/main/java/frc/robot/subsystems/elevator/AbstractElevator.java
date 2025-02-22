// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractElevator extends SubsystemBase {

  public enum TargetPosition {
    kDontCare,
    kBottom,
    kL1,
    kL2,
  }

  /** Creates a new AbstractElevator. */
  public AbstractElevator() {
  }

  public abstract void setSpeed(double percentSpeed);

  public abstract void setVoltage(double voltage);

  public abstract void setTargetPosition(TargetPosition position);

  public abstract void stop();

  public abstract void resetEncoders();

  public abstract double getPosition();

  public abstract double getVelocity();

  @Override
  public void periodic() {
    // Always invoke the base class version, unless you *know* why you shouldn't do
    // so.
    super.periodic();

    SmartDashboard.putNumber("elevator encoder position", getPosition());
    SmartDashboard.putNumber("elevator encoder velocity", getVelocity());
  }
}
