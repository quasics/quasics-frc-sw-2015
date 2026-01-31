// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDrive extends Command {
  AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_linearSpeedSupplier;
  private final Supplier<Double> m_turnSpeedSupplier;

  /** Creates a new ArcadeDrive. */

  public ArcadeDrive(Supplier<Double> linearSpeedSupplier, Supplier<Double> turnSpeedSupplier,
      AbstractDrivebase drivebase) {
    m_drivebase = drivebase;
    m_linearSpeedSupplier = linearSpeedSupplier;
    m_turnSpeedSupplier = turnSpeedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Call m_drivebase.setSpeeds()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Implement and Call m_drivebase.stop()
  }

}
