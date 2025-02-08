// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ArmRoller;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunKraken extends Command {
  ArmRoller m_ArmRoller;
  double m_speed;

  /** Creates a new RunKraken. */
  public RunKraken(ArmRoller armRoller, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmRoller = armRoller;
    m_speed = speed;

    addRequirements(armRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmRoller.setSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmRoller.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmRoller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
