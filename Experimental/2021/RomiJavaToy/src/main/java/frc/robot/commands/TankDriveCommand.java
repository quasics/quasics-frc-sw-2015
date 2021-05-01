// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.util.PowerFunction;

public class TankDriveCommand extends CommandBase {
  private RomiDrivetrain drivetrain;
  private PowerFunction leftPower;
  private PowerFunction rightPower;

  /** Creates a new TankDriveCommand. */
  public TankDriveCommand(RomiDrivetrain drivetrain, PowerFunction leftPower, PowerFunction rightPower) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(leftPower.get(), rightPower.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
