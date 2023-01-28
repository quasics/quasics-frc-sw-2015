// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class TankDrive extends CommandBase {
  public interface SpeedSupplier {
    double percent();
  }

  private final DriveBase m_driveBase;
  private final SpeedSupplier m_leftSupplier;
  private final SpeedSupplier m_rightSupplier;

  /** Creates a new TankDrive. */
  public TankDrive(DriveBase driveBase, SpeedSupplier leftSupplier, SpeedSupplier rightSupplier) {
    addRequirements(driveBase);
    m_driveBase = driveBase;
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBase.tankDrive(m_leftSupplier.percent(), m_rightSupplier.percent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveBase.tankDrive(m_leftSupplier.percent(), m_rightSupplier.percent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }
}
