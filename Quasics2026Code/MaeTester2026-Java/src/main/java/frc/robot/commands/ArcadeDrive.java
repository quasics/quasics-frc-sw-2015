// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import java.util.function.Supplier;

/**
 * Command providing support for driving the robot in "arcade drive" mode (split
 * or otherwise).
 */
public class ArcadeDrive extends Command {
  private final Drivebase m_drivebase;
  private final Supplier<Double> m_powerFunction;
  private final Supplier<Double> m_turnFunction;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivebase drivebase, Supplier<Double> powerFunction,
      Supplier<Double> turnFunction) {
    m_drivebase = drivebase;
    m_powerFunction = powerFunction;
    m_turnFunction = turnFunction;
    addRequirements(drivebase);
  }

  /** Updates the current drive settings. */
  private void updateSpeeds() {
    m_drivebase.arcadeDrive(m_powerFunction.get(), m_turnFunction.get());
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
  @Override
  public boolean isFinished() {
    return false;
  }
}
