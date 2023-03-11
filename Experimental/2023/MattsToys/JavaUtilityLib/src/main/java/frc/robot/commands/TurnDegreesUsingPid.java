// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.utils.MathUtils.approximatelyEqual;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveBaseInterface;

/**
 * Simple class to implement turning to an angle using a PID controller.
 *
 * <p>Note: this should be enhanced to use FeedForward as well.
 */
public class TurnDegreesUsingPid extends CommandBase {
  /** Default tolerance for "close enough" computations. */
  public static final double DEFAULT_TOLERANCE = 0.05;

  private final DriveBaseInterface m_driveBase;
  private final double m_angleDegrees;
  private final double m_tolerance;
  private double m_targetAngle = 0;

  // PID constants should be tuned per robot
  // TODO(mjh): Tune these for at least one of the robots! :-)
  private final double ANGULAR_P = 0.1;
  private final double ANGULAR_I = 0.01;
  private final double ANGULAR_D = 0.0;
  private final PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  /** Constructor, using the default tolerance. */
  public TurnDegreesUsingPid(DriveBaseInterface drivebase, double angleDegrees) {
    this(drivebase, angleDegrees, DEFAULT_TOLERANCE);
  }

  /**
   * Constructor.
   *
   * @param drivebase the drive base subsystem for the robot
   * @param angleDegrees the angle by which the robot should turn itself
   * @param tolerance the tolerance we're willing to accept on the angle (0.01 = 1%, etc.)
   */
  public TurnDegreesUsingPid(DriveBaseInterface drivebase, double angleDegrees, double tolerance) {
    m_driveBase = drivebase;
    m_angleDegrees = angleDegrees;
    m_tolerance = tolerance;
    addRequirements((Subsystem) m_driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetAngle = m_driveBase.getYawDegrees() + m_angleDegrees;
    m_driveBase.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate angular turn power.
    // Multiplication by -1.0 is required to ensure positive PID controller effort _increases_ yaw.
    final double rotationSpeed =
        -turnController.calculate(m_driveBase.getYawDegrees(), m_targetAngle);
    m_driveBase.arcadeDrive(0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return approximatelyEqual(m_targetAngle, m_driveBase.getYawDegrees(), m_tolerance);
  }
}
