// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.RealDrivebase;
import java.util.function.Supplier;

// CODE_REVIEW/FIXME: If we're not going to use tank drive for the robot, then we
// should probably just remove this command.
public class TankDrive extends Command {
  private final RealDrivebase m_drivebase;
  private final Supplier<Double> m_leftSupplier;
  private final Supplier<Double> m_rightSupplier;

  /** Creates a new TankDrive. */
  public TankDrive(
      RealDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    m_drivebase = drivebase;
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // TODO: Finish implementing this function, once IDrivebase supports tank drive.
  private void updateSpeeds() {
    final double leftInput = m_leftSupplier.get();
    final double rightInput = m_rightSupplier.get();

    double leftSpeed = leftInput;
    double rightSpeed = rightInput;
    // m_drivebase.setSpeeds(leftSpeed, rightSpeed);
  }
}
