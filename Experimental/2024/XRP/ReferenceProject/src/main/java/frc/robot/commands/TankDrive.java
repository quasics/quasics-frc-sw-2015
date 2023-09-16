// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class TankDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_leftSpeedSupplier;
  private final Supplier<Double> m_rightRotateSupplier;

  /** Creates a new TankDrive. */
  public TankDrive(
    Drivetrain drivetrain,
    Supplier<Double> leftSpeedSupplier,
    Supplier<Double> rightRotateSupplier) {
  m_drivetrain = drivetrain;
  m_leftSpeedSupplier = leftSpeedSupplier;
  m_rightRotateSupplier = rightRotateSupplier;
  addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_leftSpeedSupplier.get(), m_leftSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
