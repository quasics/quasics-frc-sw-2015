// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.TargetPositioningUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHub extends Command {
  final static Angle tolerance = Degrees.of(1);
  IDrivebase m_drivebase;
  Rotation2d m_goalAngle;
  final PIDController m_pid;

  /** Creates a new AlignToHub. */
  public AlignToHub(IDrivebase drivebase) {
    m_drivebase = drivebase;
    m_pid = new PIDController(0.002, 0, 0);
    m_pid.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goalAngle = TargetPositioningUtils.getAngleToHubCenter(m_drivebase.getEstimatedPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentAngle = m_drivebase.getEstimatedPose().getRotation();
    Rotation2d error = m_goalAngle.minus(currentAngle);
    double rotationPercent = m_pid.calculate(0.0, error.getDegrees());
    m_drivebase.arcadeDrive(0, rotationPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d currentAngle = m_drivebase.getEstimatedPose().getRotation();
    Rotation2d error = m_goalAngle.minus(currentAngle);
    return error.getMeasure().abs(Degrees) <= tolerance.abs(Degrees);
  }
}
