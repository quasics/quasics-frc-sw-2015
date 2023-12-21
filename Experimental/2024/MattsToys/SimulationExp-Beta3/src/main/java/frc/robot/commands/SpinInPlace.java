// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

public class SpinInPlace extends Command {
  final private AbstractDrivebase m_drivebase;

  /** Creates a new SpinInPlace. */
  public SpinInPlace(AbstractDrivebase drivebase) {
    m_drivebase = drivebase;
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.arcadeDrive(0, AbstractDrivebase.MAX_ANGULAR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.arcadeDrive(0, AbstractDrivebase.MAX_ANGULAR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
