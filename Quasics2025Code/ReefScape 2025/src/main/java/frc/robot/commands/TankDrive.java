// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.RealDrivebase;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TankDrive extends Command {
  private final RealDrivebase m_drivebase;
  private final Supplier<Double> m_leftSupplier;
  private final Supplier<Double> m_rightSupplier;
  /** Creates a new TankDrive. */
  public TankDrive(RealDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
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

  private void updateSpeeds() {
    final double leftInput = m_leftSupplier.get();
    final double rightInput = m_rightSupplier.get();
    
    double leftSpeed = leftInput;
    double rightSpeed = rightInput;
    // TODO: add tank drive support to IDrivebase
   // m_drivebase.setSpeeds(leftSpeed, rightSpeed);
  }
}
