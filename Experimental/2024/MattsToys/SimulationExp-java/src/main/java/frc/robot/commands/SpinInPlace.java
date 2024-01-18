// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

public class SpinInPlace extends Command {
  final private AbstractDrivebase m_drivebase;
  final private static Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);

  /** Creates a new SpinInPlace. */
  public SpinInPlace(AbstractDrivebase drivebase) {
    m_drivebase = drivebase;
    addRequirements(m_drivebase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.arcadeDrive(ZERO_MPS, AbstractDrivebase.MAX_ANGULAR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
