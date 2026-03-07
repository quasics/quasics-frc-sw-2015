// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.util.HeadingUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHeading extends Command {
  final static Angle TOLERANCE = Degrees.of(1);

  final IDrivebasePlus m_drivebase;
  final Rotation2d m_targetHeading;
  final PIDController m_pid;

  /** Creates a new TurnToHeading. */
  public TurnToHeading(IDrivebasePlus drivebase, Rotation2d targetHeading) {
    m_drivebase = drivebase;
    m_targetHeading = targetHeading;

    // Note that PID values need to be tuned to a robot. This could be done by (for
    // example) running angular profiling using SysID.
    m_pid = new PIDController(0.002, 0, 0.0);

    addRequirements(m_drivebase.asSubsystem());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d error = HeadingUtils.getRotationError(m_drivebase.getEstimatedPose(), m_targetHeading);

    // Calculate rotation speed based on the error in degrees
    double rotationOutput = m_pid.calculate(0, error.getDegrees());

    // Apply to your drive motors (e.g., arcadeDrive or chassisSpeeds)
    m_drivebase.driveArcade(0, rotationOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d error = HeadingUtils.getRotationError(m_drivebase.getEstimatedPose(), m_targetHeading);
    System.out.printf("Error: %.4f (%.4f abs)\n", error.getMeasure().in(Degrees), error.getMeasure().abs(Degrees));
    return error.getMeasure().abs(Degrees) <= TOLERANCE.abs(Degrees);
  }
}
