// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Implements a simple "tank drive" command.
 * 
 * Suggestion from WPILib template: "You should consider using the more terse
 * Command factories API instead."
 * 
 * @see https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TankDrive extends Command {
  private final AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_leftSupplier;
  private final Supplier<Double> m_rightSupplier;

  /** Creates a new TankDrive. */
  public TankDrive(AbstractDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    m_drivebase = drivebase;
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;

    addRequirements(m_drivebase);
  }

  private void updateSpeeds() {
    m_drivebase.tankDrive(m_leftSupplier.get(), m_rightSupplier.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  //
  // Note that this command never ends, unless it's interrupted because some other
  // command requires the subsystem. (This is a common pattern for default
  // commands; we could leave this function out, since the Command base class
  // already does exactly this, but it's left here to be explicit as an example.)
  @Override
  public boolean isFinished() {
    return false;
  }
}
