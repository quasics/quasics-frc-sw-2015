// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

public class BaseChoreoTrajectoryCommand extends Command {
  static final boolean SHOULD_MIRROR_POSES_FOR_RED_ALLIANCE = false;

  private final Timer m_timer = new Timer();
  private final LTVUnicycleController m_controller = new LTVUnicycleController(0.02);
  private final IDrivebasePlus m_drivebase;
  private final Optional<Trajectory<DifferentialSample>> m_trajectory;
  private boolean m_mirrorPoses;

  /**
   * Constructor.
   * 
   * @param trajectoryName file name for the trajectory (Choreo will assume the
   *                       file path is known)
   * @param drivebase      drive base being controlled
   */
  public BaseChoreoTrajectoryCommand(String trajectoryName, IDrivebasePlus drivebase) {
    m_drivebase = drivebase;
    m_trajectory = Choreo.loadTrajectory(trajectoryName);

    addRequirements(m_drivebase.asSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_trajectory.isPresent()) {
      return;
    }

    m_mirrorPoses = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)
        && SHOULD_MIRROR_POSES_FOR_RED_ALLIANCE;

    // Get the initial pose of the trajectory
    Optional<Pose2d> initialPose = m_trajectory.get().getInitialPose(m_mirrorPoses);

    if (initialPose.isPresent()) {
      m_drivebase.resetOdometry(initialPose.get());
      // TODO: Consider some of the following options.
      // 1) Providing an indication of how closely the initial pose matches the
      // current drivebase pose.
      // 2) Adapting the trajectory (somehow) to the current pose.
      // 3) (Longshot) First, get us to the targeted initial pose, and then....
    }

    // Reset and start the timer when the autonomous period begins
    m_timer.restart();
  }

  private void followTrajectory(DifferentialSample sample) {
    // Get the current pose of the robot
    Pose2d pose = m_drivebase.getEstimatedPose();

    // Get the velocity feedforward specified by the sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = m_controller.calculate(
        pose,
        sample.getPose(),
        ff.vxMetersPerSecond,
        ff.omegaRadiansPerSecond);

    // Apply the generated speeds
    m_drivebase.driveTankWithPID(speeds);
  }

  @Override
  public void execute() {
    if (m_timer.isRunning() && m_trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<DifferentialSample> sample = m_trajectory.get().sampleAt(m_timer.get(), m_mirrorPoses);
      if (sample.isPresent()) {
        followTrajectory(sample.get());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_timer.isRunning()
        || m_trajectory.isEmpty()
        || m_timer.hasElapsed(m_trajectory.get().getTotalTime());
  }

  // public void followTrajectory(DifferentialSample sample) {
  // // Get the current pose of the robot
  // Pose2d pose = getPose();

  // // Get the velocity feedforward specified by the sample
  // ChassisSpeeds ff = sample.getChassisSpeeds();

  // // Generate the next speeds for the robot
  // ChassisSpeeds speeds = controller.calculate(
  // pose,
  // sample.getPose(),
  // ff.vxMetersPerSecond,
  // ff.omegaRadiansPerSecond);

  // // Apply the generated speeds
  // drive(speeds);

  // // Or, if you don't drive via ChassisSpeeds
  // DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
  // //
  // drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  // }
}
