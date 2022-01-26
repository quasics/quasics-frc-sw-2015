// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterSetVelocity extends CommandBase {
  private final Shooter shooter;
  private final double speedRpm;

  /**
   * Creates a new ShooterSetVelocity.
   * 
   * Note that this class assumes that the velocity specified here, and that
   * returned by the shooter, is expressed in RPM.
   * 
   * @see frc.robot.subsystems.Shooter#getSpeed()
   */
  public ShooterSetVelocity(Shooter shooter, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    this.shooter = shooter;
    this.speedRpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(speedRpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
