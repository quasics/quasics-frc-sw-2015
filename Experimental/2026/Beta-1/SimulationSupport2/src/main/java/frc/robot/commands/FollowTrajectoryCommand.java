// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Command to follow a specified (robot-relative) trajectory.
 */
public class FollowTrajectoryCommand extends Command {
  /** Drive base being controlled. */
  final IDrivebasePlus m_drivebase;

  /** Robot-relative trajectory to be followed. */
  final Trajectory m_baseTrajectory;

  /** Timer used to identify samples from the trajectory. */
  final Timer m_timer = new Timer();

  /** Feedforward configuration used for the drivebase. */
  final SimpleMotorFeedforward m_feedforward;

  /** PID controller used to adjust power to the left side of the drivebase. */
  final PIDController m_leftPID;

  /** PID controller used to adjust power to the right side of the drivebase. */
  final PIDController m_rightPID;

  /** Field-relative version of m_baseTrajectory, [re]computed when the command starts running. */
  Trajectory m_currentTrajectory;

  /**
   * Unicycle controller, used to calculate wheel speeds along the way.
   *
   * Notes:
   * <ul>
   * <li>We should consider updating the controller's allocation to also specify the maximum
   * velocity (in m/s).
   *
   * <li>This is the replacement for the RamseteController (deprecated in 2025).
   * </ul>
   */
  final LTVUnicycleController m_controller = new LTVUnicycleController(0.2);

  /**
   * Constructor.
   *
   * @param drivebase   drivebase being controlled
   * @param trajectory  robot-relative trajectory to be followed
   */
  public FollowTrajectoryCommand(IDrivebasePlus drivebase, Trajectory trajectory) {
    m_drivebase = drivebase;
    m_baseTrajectory = trajectory;
    m_feedforward =
        new SimpleMotorFeedforward(m_drivebase.getKs(), m_drivebase.getKv(), m_drivebase.getKa());
    m_leftPID = new PIDController(m_drivebase.getKp(), 0, 0);
    m_rightPID = new PIDController(m_drivebase.getKp(), 0, 0);
    addRequirements(m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_leftPID.reset();
    m_rightPID.reset();

    // Convert the base trajectory into something relative to the robot's initial pose when the
    // command starts running.
    Transform2d transform = new Transform2d(new Pose2d(), m_drivebase.getEstimatedPose());
    m_currentTrajectory = m_baseTrajectory.transformBy(transform);
  }

  @Override
  public void execute() {
    // Calculate how fast we should be moving at this point along the trajectory
    double elapsed = m_timer.get();
    var referencePosition = m_currentTrajectory.sample(elapsed);

    //
    // OK, let's make that happen.
    //

    // 1. Compute the wheel speeds required at this point.
    ChassisSpeeds newSpeeds =
        m_controller.calculate(m_drivebase.getEstimatedPose(), referencePosition);
    DifferentialDriveWheelSpeeds wheelSpeeds = Drivebase.KINEMATICS.toWheelSpeeds(newSpeeds);

    // 2. Calculate Feedforward (Predictive Volts)
    // kV is Volts per m/s, so (m/s * kV) = Volts
    double leftFFVolts = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFFVolts = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    // 3. Calculate PID (Correction Volts)
    // Since the Kp is assumed to be tuned to output Volts, this result is also in Volts
    double leftPIDVolts =
        m_leftPID.calculate(m_drivebase.getLeftVelocity().in(MetersPerSecond), // Current m/s
            wheelSpeeds.leftMetersPerSecond // Target m/s
        );
    double rightPIDVolts =
        m_rightPID.calculate(m_drivebase.getRightVelocity().in(MetersPerSecond), // Current m/s
            wheelSpeeds.rightMetersPerSecond // Target m/s
        );

    // 4. Combine and apply
    m_drivebase.tankDriveVolts(
        Volts.of(leftFFVolts + leftPIDVolts), Volts.of(rightFFVolts + rightPIDVolts));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_currentTrajectory.getTotalTimeSeconds());
  }
}
