// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

/** A command that controls the robot using arcade-style driving. */
public class ArcadeDrive extends Command {
  /** The drivebase subsystem to use. */
  IDrivebase drivebase;
  /** Supplier for forward speed. */
  Supplier<Double> speedSupplier;
  /** Supplier for rotation rate. */
  Supplier<Double> rotationSupplier;

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param drivebase        drivebase subsystem to use
   * @param speedSupplier    supplier for forward speed
   * @param rotationSupplier supplier for rotation rate
   */
  public ArcadeDrive(IDrivebase drivebase, Supplier<Double> speedSupplier,
                     Supplier<Double> rotationSupplier) {
    this.drivebase = drivebase;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    drivebase.driveArcade(speedSupplier.get(), rotationSupplier.get());
  }

  @Override
  public void execute() {
    drivebase.driveArcade(speedSupplier.get(), rotationSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }
}
