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

/**
 * Base class for commands that handle turning the robot to some
 * (field-relative) heading.
 */
public abstract class TurnToHeadingBase extends Command {
  /** Maximum error we'll accept as "OK". */
  final static Angle TOLERANCE = Degrees.of(1);

  /** Drivebase being controlled. */
  final IDrivebasePlus m_drivebase;

  /** Target heading (populated in initialize()). */
  Rotation2d m_targetHeading;

  /** PID controller used to adjust speed for turning. */
  final PIDController m_pid;

  /**
   * Constructor.
   * 
   * @param drivebase the drivebase being controlled
   */
  public TurnToHeadingBase(IDrivebasePlus drivebase) {
    m_drivebase = drivebase;

    // Note that PID values need to be tuned to a robot. This could be done by (for
    // example) running angular profiling using SysID.
    m_pid = new PIDController(0.002, 0, 0.0);

    addRequirements(drivebase.asSubsystem());
  }

  /**
   * Returns the heading to which the robot should turn. (Called from
   * initialize().)
   */
  abstract protected Rotation2d getTargetHeading();

  @Override
  public void initialize() {
    m_targetHeading = getTargetHeading();
  }

  @Override
  public void execute() {
    // Calculate rotation error (i.e., how far off are we from the target heading)
    Rotation2d error = HeadingUtils.getRotationError(m_drivebase.getEstimatedPose(), m_targetHeading);

    // Calculate rotation speed based on the error in degrees
    double rotationOutput = m_pid.calculate(0, error.getDegrees());

    // Apply to your drive motors (e.g., arcadeDrive or chassisSpeeds)
    m_drivebase.driveArcade(0, rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    Rotation2d error = HeadingUtils.getRotationError(m_drivebase.getEstimatedPose(), m_targetHeading);
    return error.getMeasure().abs(Degrees) <= TOLERANCE.abs(Degrees);
  }
}
