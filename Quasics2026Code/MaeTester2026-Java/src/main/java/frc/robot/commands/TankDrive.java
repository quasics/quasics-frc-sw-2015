// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import java.util.function.Supplier;

/**
 * Command providing support for driving the robot in "tank drive" mode.
 */
public class TankDrive extends Command {
  private final Drivebase m_drivebase;
  private final Supplier<Double> m_leftPowerFunction;
  private final Supplier<Double> m_rightPowerFunction;

  /** Creates a new TankDrive. */
  public TankDrive(Drivebase drivebase, Supplier<Double> leftPowerFunction,
      Supplier<Double> rightPowerFunction) {
    m_drivebase = drivebase;
    m_leftPowerFunction = leftPowerFunction;
    m_rightPowerFunction = rightPowerFunction;
    addRequirements(drivebase);
  }

  /** Updates the current drive settings. */
  private void updateSpeeds() {
    m_drivebase.setMotorSpeed(
        m_leftPowerFunction.get(), m_rightPowerFunction.get());
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
}
