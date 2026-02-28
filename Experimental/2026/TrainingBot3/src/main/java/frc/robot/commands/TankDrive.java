// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;
import java.util.function.Supplier;

/**
 * Implements a simple "tank drive" command.
 *
 * Suggestion from WPILib template: "You should consider using the more terse
 * Command factories API instead."
 *
 * @see
 *      https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TankDrive extends Command {
  /**
   * The drive base we're working with. (Abstract, so that we can work with either
   * the real hardware or simulation.)
   */
  private final AbstractDrivebase m_drivebase;

  /**
   * Used to get a "double" value, to use in controlling left-side wheel speeds.
   * 
   * @see https://docs.oracle.com/javase/8/docs/api/java/util/function/Supplier.html
   */
  private final Supplier<Double> m_leftSupplier;

  /**
   * Used to get a "double" value, to use in controlling right-side wheel speeds.
   * 
   * @see https://docs.oracle.com/javase/8/docs/api/java/util/function/Supplier.html
   */
  private final Supplier<Double> m_rightSupplier;

  /**
   * Creates a new TankDrive command.
   * 
   * @param drivebase     the drive base object we're actually using to control
   *                      the robot
   * @param leftSupplier  the Supplier object providing left-side speed values
   * @param rightSupplier the Supplier object providing right-side speed values
   */
  public TankDrive(
      AbstractDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    m_drivebase = drivebase;
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;

    addRequirements(m_drivebase);
  }

  /** Helper method to set the left/right speed values. */
  private void updateSpeeds() {
    m_drivebase.tankDrive(m_leftSupplier.get(), m_rightSupplier.get());
  }

  // Called when the command is initially scheduled. ("Start whatever the command
  // does.")
  @Override
  public void initialize() {
    updateSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled. ("Make
  // any changes that are needed.")
  @Override
  public void execute() {
    updateSpeeds();
  }

  // Called once the command ends or is interrupted. ("Do any shutdown/cleanup
  // that's needed when the command is done.")
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  /**
   * Standard "Command" class method that signals when the command has finished
   * running.
   * 
   * Note that this particular command *never* ends, unless it's interrupted
   * because some other command requires the subsystem.
   * 
   * This is a common pattern for default commands; we could leave this function
   * out, since the Command base class already does exactly this, but it's left
   * here to be an explicit example.)
   * 
   * @return true if the command has finished doing its job
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
