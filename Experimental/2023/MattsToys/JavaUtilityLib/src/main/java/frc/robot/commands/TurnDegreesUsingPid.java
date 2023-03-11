// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveBaseInterface;
import frc.robot.utils.MathUtils;

/**
 * Simple class to implement turning to an angle using a PID controller.
 *
 * <p>Note: this really should be enhanced to use FeedForward as well.
 */
public class TurnDegreesUsingPid extends CommandBase {
  /**
   * TODO(mjh): Decide if we want to use radians or degrees for PID computation. (I think we want
   * degrees, but I'm leaving the option open a little longer.)
   */
  static final boolean USE_RADIANS_FOR_PID = false;

  /**
   * If true, PID control will be used simply to try to slow us down as we approach the target
   * angle; once we exceed it, we'll stop.
   *
   * <p>If false, we'll try to center in on the angle, within the established tolerance. This means
   * that the command may continue running indefinitely, unless the PID parameters are well-tuned,
   * or unless the tolerance interval is sufficiently wide.
   */
  static final boolean STOP_WHEN_OVER = false;

  /** Default tolerance for "close enough" computations. */
  public static final double DEFAULT_TOLERANCE_DEGREES = 2.0;

  // TODO(mjh): Come up with default PID params that feel at least vaguely real.
  private static final double DEFAULT_ANGULAR_P = 0.1;
  private static final double DEFAULT_ANGULAR_I = 0.002;
  private static final double DEFAULT_ANGULAR_D = 0.0;

  /** Drive base subsystem for the robot. */
  private final DriveBaseInterface m_driveBase;

  /** Number of degrees by which we should turn when command is executed. */
  private final double m_angleDegrees;

  /**
   * Acceptable tolerance for the turn (within this many degrees).
   *
   * <p>Note: this is only used when `STOP_WHEN_OVER` is false.
   */
  private final double m_toleranceDegrees;

  /** PID controller used to compute speeds while command is executing. */
  private final PIDController m_turnController;

  /** Target angle to which we want to turn. (Set during initialize().) */
  private double m_targetAngleDegrees = 0;

  /** Last speed obtained from the PIDController, used for debugging output. */
  private double m_lastComputedSpeed = 0;

  /** Constructor, using the default tolerance. */
  public TurnDegreesUsingPid(DriveBaseInterface drivebase, double angleDegrees) {
    this(drivebase, angleDegrees, DEFAULT_TOLERANCE_DEGREES);
  }

  /**
   * Constructor.
   *
   * @param drivebase the drive base subsystem for the robot
   * @param angleDegrees the angle (in degrees) by which the robot should turn itself
   * @param tolerance the "within X degrees" tolerance we're willing to accept on the angle
   */
  public TurnDegreesUsingPid(DriveBaseInterface drivebase, double angleDegrees, double tolerance) {
    this(
        drivebase,
        angleDegrees,
        tolerance,
        DEFAULT_ANGULAR_P,
        DEFAULT_ANGULAR_I,
        DEFAULT_ANGULAR_D);
  }

  /**
   * Constructor.
   *
   * <p>Note: PID constants should be tuned per robot.
   *
   * <p>TODO(mjh): Tune these for at least one of the robots! :-)
   *
   * @param drivebase the drive base subsystem for the robot
   * @param angleDegrees the angle (in degrees) by which the robot should turn itself
   * @param tolerance the "within X degrees" tolerance we're willing to accept on the angle
   * @param kP "kP" value for the PID controller
   * @param kI "kI" value for the PID controller
   * @param kD "kD" value for the PID controller
   */
  public TurnDegreesUsingPid(
      DriveBaseInterface drivebase,
      double angleDegrees,
      double tolerance,
      double kP,
      double kI,
      double kD) {
    m_driveBase = drivebase;
    m_angleDegrees = angleDegrees;
    m_toleranceDegrees = tolerance;
    m_turnController = new PIDController(kP, kI, kD);
    addRequirements((Subsystem) m_driveBase);
  }

  @Override
  public void initialize() {
    m_targetAngleDegrees = m_driveBase.getYawDegrees() + m_angleDegrees;
    m_driveBase.arcadeDrive(0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }

  @Override
  public void execute() {
    // Calculate angular turn power.
    // Note that multiplication by -1.0 is required to ensure positive PID controller effort
    // _increases_ yaw.
    double computedRotationSpeed;
    if (USE_RADIANS_FOR_PID) {
      computedRotationSpeed =
          -m_turnController.calculate(
              Units.degreesToRadians(m_driveBase.getYawDegrees()),
              Units.degreesToRadians(m_targetAngleDegrees));
    } else {
      computedRotationSpeed =
          -m_turnController.calculate(m_driveBase.getYawDegrees(), m_targetAngleDegrees);
    }
    m_lastComputedSpeed = computedRotationSpeed;
    final double rotationSpeed =
        computedRotationSpeed; // Math.max(Math.min(computedRotationSpeed, 0.5), -0.5);
    m_driveBase.arcadeDrive(0, rotationSpeed);
  }

  @Override
  public boolean isFinished() {
    final double currentAngleDegrees = m_driveBase.getYawDegrees();
    System.err.println(
        "Target: "
            + m_targetAngleDegrees
            + ", current: "
            + currentAngleDegrees
            + ", last speed: "
            + m_lastComputedSpeed);
    if (STOP_WHEN_OVER) {
      // Simply stop when we exceed the target angle.
      return (m_angleDegrees == 0)
          || (m_angleDegrees < 0 && m_targetAngleDegrees > currentAngleDegrees)
          || (m_angleDegrees >= 0 && m_targetAngleDegrees < currentAngleDegrees);
    } else {
      // Try to make sure that we're within acceptable tolerance.  If not, let PID continue driving
      // us towards it.
      return MathUtils.withinTolerance(
          m_targetAngleDegrees, currentAngleDegrees, m_toleranceDegrees);
    }
  }
}
