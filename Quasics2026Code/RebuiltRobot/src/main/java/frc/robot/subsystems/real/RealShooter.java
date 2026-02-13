// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IShooter;

/**
 * Subsystem for controlling the shooter mechanism, which is used to shoot balls
 * into the Hub.
 * 
 * FINDME(Rylie): Some suggested reading for this subsystem:
 * * Straightforward PID control for flywheels:
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
 */
public class RealShooter extends SubsystemBase implements IShooter {
  /** Creates a new RealShooter. */
  public RealShooter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
