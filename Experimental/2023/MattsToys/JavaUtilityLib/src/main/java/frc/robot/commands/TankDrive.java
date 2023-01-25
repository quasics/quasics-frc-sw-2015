// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AbstractDriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
  private final AbstractDriveBase m_driveBase;
  private final Supplier<Double> m_leftSpeedSupplier;
  private final Supplier<Double> m_rightSpeedSupplier;

  /**
   * Creates a new TankDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param leftSpeedSupplier Lambda supplier of left speed
   * @param rightSpeedSupplier Lambda supplier of right speed
   */
  public TankDrive(
      AbstractDriveBase drivetrain,
      Supplier<Double> leftSpeedSupplier,
      Supplier<Double> rightSpeedSupplier) {
    m_driveBase = drivetrain;
    m_leftSpeedSupplier = leftSpeedSupplier;
    m_rightSpeedSupplier = rightSpeedSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveBase.tankDrive(m_leftSpeedSupplier.get(), m_rightSpeedSupplier.get());
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
