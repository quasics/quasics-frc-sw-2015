// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeployment;

/**
 * Simple command to run the intake deployment (winch) under operator control.
 */
public class RunWinch extends CommandBase {
  final IntakeDeployment m_intake;
  final double m_speed;

  /**
   * Creates a new RunWinch command.
   * 
   * @param intakeDeployment the "intake deployment" (i.e., winch) subsystem
   * @param speed            the speed (motor %) at which the intake should be
   *                         moving (-1.0 to +1.0)
   */
  public RunWinch(IntakeDeployment intake, double speed) {
    this.m_intake = intake;
    this.m_speed = Math.max(-1, Math.min(+1, speed));
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setMotorSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
