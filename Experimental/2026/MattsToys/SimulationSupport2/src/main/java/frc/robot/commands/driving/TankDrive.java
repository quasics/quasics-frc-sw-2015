// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

/** A command that controls the robot using tank-style driving. */
public class TankDrive extends Command {
  /** The drivebase subsystem to use. */
  IDrivebase drivebase;
  /** Supplier for left speed. */
  Supplier<Double> leftSpeedSupplier;
  /** Supplier for right speed. */
  Supplier<Double> rightSpeedSupplier;

  /**
   * Creates a new TankDrive command.
   *
   * @param drivebase          drivebase subsystem to use
   * @param leftSpeedSupplier  supplier for left speed
   * @param rightSpeedSupplier supplier for right speed
   */
  public TankDrive(IDrivebase drivebase, Supplier<Double> leftSpeedSupplier,
      Supplier<Double> rightSpeedSupplier) {
    this.drivebase = drivebase;
    this.leftSpeedSupplier = leftSpeedSupplier;
    this.rightSpeedSupplier = rightSpeedSupplier;

    addRequirements(drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    drivebase.driveTank(leftSpeedSupplier.get(), rightSpeedSupplier.get());
  }

  @Override
  public void execute() {
    drivebase.driveTank(leftSpeedSupplier.get(), rightSpeedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }
}
