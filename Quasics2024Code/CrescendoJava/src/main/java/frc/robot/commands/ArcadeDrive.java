// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class ArcadeDrive extends Command {
  private final Drivebase m_drivebase;
  private final Supplier<Double> m_speedSupplier;
  private final Supplier<Double> m_rotationSupplier;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    m_speedSupplier = leftSupplier;
    m_rotationSupplier = rightSupplier;
    m_drivebase = drivebase;

    addRequirements(drivebase);
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

  private void updateSpeeds() {
    final double leftInput = m_speedSupplier.get();
    final double rightInput = m_rotationSupplier.get();

    Measure<Velocity<Distance>> forwardSpeed = Drivebase.MAX_SPEED.times(leftInput);
    Measure<Velocity<Angle>> rotationSpeed = Drivebase.MAX_ANGULAR_SPEED.times(rightInput);

    m_drivebase.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
