// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

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
    m_drivebase.arcadeDrive(MetersPerSecond.of(m_linearSpeedSupplier.get()),
        RadiansPerSecond.of(m_turnSpeedSupplier.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Implement and Call m_drivebase.stop()
  }

}
