// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ArcadeDrive extends Command {
  private final AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_speedSupplier;
  private final Supplier<Double> m_rotationSupplier;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(
      AbstractDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  private void updateSpeeds() {
    final double leftInput = m_speedSupplier.get();
    final double rightInput = m_rotationSupplier.get();

    LinearVelocity forwardSpeed = AbstractDrivebase.MAX_SPEED.times(leftInput);
    AngularVelocity rotationSpeed = AbstractDrivebase.MAX_ANGULAR_SPEED.times(rightInput);

    m_drivebase.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}
