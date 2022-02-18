// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.utils.DrivePowerSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {

  final DriveBase driveBase;
  final DrivePowerSupplier leftPowerSupplier;
  final DrivePowerSupplier rightPowerSupplier;

  /** Creates a new TankDrive. */
  public TankDrive(DriveBase driveBase, DrivePowerSupplier leftPowerSupplier, DrivePowerSupplier rightPowerSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);

    this.driveBase = driveBase;
    this.leftPowerSupplier = leftPowerSupplier;
    this.rightPowerSupplier = rightPowerSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBase.tankDrive(leftPowerSupplier.get(), rightPowerSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.tankDrive(leftPowerSupplier.get(), rightPowerSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
