// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Basic implementation of chunks of the IDrivebase interface. Setup/retrieval
 * of the underlying hardware is left to derived classes.
 * 
 * TODO: Consider adding "mode" info, to allow switching between PID and "manual
 * control".
 */
public abstract class AbstractDrivebase extends SubsystemBase implements IDrivebase {
  /** Controls if data should be logged to the dashboard. */
  final static boolean LOG_TO_DASHBOARD = true;

  /** Kinematics definition for this drive base. */
  final protected DifferentialDriveKinematics m_kinematics;

  // PID/FF calculators
  /** PID controller for left side motors. */
  final protected PIDController m_leftPidController;
  /** PID controller for right side motors. */
  final protected PIDController m_rightPidController;
  /** Feedforward controller for drive base. */
  final protected DifferentialDriveFeedforward m_feedforward;

  /** Unicycle controller for use with trajectory-following. */
  final LTVUnicycleController m_unicycleController = new LTVUnicycleController(0.02);

  /**
   * Constructor.
   * 
   * @param config configuration for the targeted robot (PID/FF constants, etc.)
   */
  protected AbstractDrivebase(RobotConfig config) {
    setName(SUBSYSTEM_NAME);
    final var driveConfig = config.drive();

    final double trackWidthMeters = driveConfig.trackWidth().in(Meters);

    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);

    // PID and FF setup
    m_leftPidController = new PIDController(driveConfig.pid().kP(), driveConfig.pid().kI(), driveConfig.pid().kD());
    m_rightPidController = new PIDController(driveConfig.pid().kP(), driveConfig.pid().kI(), driveConfig.pid().kD());
    m_feedforward = new DifferentialDriveFeedforward(
        driveConfig.feedForward().linear().kV().in(Volts), driveConfig.feedForward().linear().kA(),
        driveConfig.feedForward().angular().kV().in(Volts),
        driveConfig.feedForward().angular().kA());
  }

  @Override
  public void periodic() {
    super.periodic();

    updateOdometry();

    publishData();
  }

  @Override
  public void driveWithPid(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // var leftStabilized =
    // wheelSpeedsDeadband.limit(wheelSpeeds.leftMetersPerSecond);
    // var rightStabilized =
    // wheelSpeedsDeadband.limit(wheelSpeeds.rightMetersPerSecond);
    // logValue("leftStable", leftStabilized);
    // logValue("rightStable", rightStabilized);

    // Figure out the voltages we should need at the target speeds.
    final var feedforwardVolts = m_feedforward.calculate(getLeftVelocity().in(MetersPerSecond),
        wheelSpeeds.leftMetersPerSecond, getRightVelocity().in(MetersPerSecond),
        wheelSpeeds.rightMetersPerSecond, 0.020);
    logValue("FF left", feedforwardVolts.left);
    logValue("FF right", feedforwardVolts.right);

    // Figure out the deltas, based on our current speed vs. the target speeds.
    double leftPidOutput = m_leftPidController.calculate(
        getLeftVelocity().in(MetersPerSecond), wheelSpeeds.leftMetersPerSecond);
    double rightPidOutput = m_rightPidController.calculate(
        getRightVelocity().in(MetersPerSecond), wheelSpeeds.rightMetersPerSecond);
    logValue("leftPid", leftPidOutput);
    logValue("rightPid", rightPidOutput);

    // OK, apply those to the actual hardware.
    setMotorVoltages(Volts.of(feedforwardVolts.left + leftPidOutput),
        Volts.of(feedforwardVolts.right + rightPidOutput));
  }

  @Override
  public void resetPose(Pose2d pose) {
    final Distance leftPosition = getLeftPosition();
    final Distance rightPosition = getRightPosition();
    final Rotation2d position = getGyro().getRotation2d();

    getOdometry().resetPosition(position, leftPosition, rightPosition, pose);
    getPoseEstimator().resetPosition(position, leftPosition.in(Meters), rightPosition.in(Meters), pose);

    // TODO: Should probably clear position data from Vision when we're running in
    // the simulator, since we could be jumping significantly in a way that the
    // vision stuff won't pick up (unlike in real-world movement).
  }

  @Override
  public Pose2d getPose() {
    return getOdometry().getPoseMeters();
  }

  @Override
  public Pose2d getEstimatedPose() {
    return getPoseEstimator().getEstimatedPosition();
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @Override
  public void followTrajectory(DifferentialSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Get the velocity feedforward specified by the sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = m_unicycleController.calculate(
        pose, sample.getPose(), ff.vxMetersPerSecond, ff.omegaRadiansPerSecond);

    // Apply the generated speeds
    driveWithPid(speeds);
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Internal helper methods
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Update the odometry/pose estimation, based on current sensor data.
   */
  private void updateOdometry() {
    final DifferentialDriveOdometry odometry = getOdometry();
    final DifferentialDrivePoseEstimator estimator = getPoseEstimator();
    if (odometry == null && estimator == null) {
      // Nothing to be updated.
      return;
    }

    final Rotation2d rotation = getGyro().getRotation2d();
    final double leftDistanceMeters = getLeftEncoder().getPosition().in(Meters);
    final double rightDistanceMeters = getRightEncoder().getPosition().in(Meters);

    if (odometry != null) {
      odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
    }

    if (estimator != null) {
      estimator.update(rotation, leftDistanceMeters, rightDistanceMeters);

      // If an estimated position has been posted by the vision subsystem, integrate
      // it into our estimate. (Note that some sources suggest *not* doing this while
      // the robot is in motion, since that's when you'll have the most significant
      // error introduced into the images.)
      Optional<Object> optionalPose = BulletinBoard.common.getValue(IVision.VISION_POSE_KEY, Pose2d.class);
      optionalPose.ifPresent(poseObject -> {
        BulletinBoard.common.getValue(IVision.VISION_TIMESTAMP_KEY, Double.class)
            .ifPresent(
                timestampObject -> estimator.addVisionMeasurement((Pose2d) poseObject, (Double) timestampObject));
      });
    }
  }

  /** Share the current pose with other subsystems (e.g., vision). */
  private void publishData() {
    final Pose2d currentPose = getPose();
    BulletinBoard.common.updateValue(POSE_KEY, currentPose);
    BulletinBoard.common.updateValue(ESTIMATED_POSE_KEY, getEstimatedPose());

    // Update published field simulation data. We're doing this here (in the
    // periodic function, rather than in the simulationPeriodic function) because we
    // want to take advantage of the fact that the odometry has just been updated.
    //
    // When we move stuff into a base class, the code above would be there, and this
    // would be in the overridden periodic function for this (simulation-specific)
    // class.
    SmartDashboard.putNumber("X", currentPose.getX());
    SmartDashboard.putNumber("Y", currentPose.getY());

    // Push our PID info out to the dashboard.
    SmartDashboard.putData("Drive pid (L)", m_leftPidController);
    SmartDashboard.putData("Drive pid (R)", m_rightPidController);
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Utility logging methods
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Logs the specified data to the dashboard iff LOG_TO_SMART_DASHBOARD is true.
   * 
   * @param label label for the value
   * @param val   value to be shown
   */
  protected void logValue(String label, double val) {
    if (LOG_TO_DASHBOARD) {
      SmartDashboard.putNumber(label, val);
    }
  }

  /**
   * Logs the specified measure to the dashboard iff LOG_TO_SMART_DASHBOARD is
   * true.
   * 
   * @param label label for the value (will have " (X)" appended, where "X" will
   *              reflect the measure's underlying units)
   * @param val   value to be shown
   */
  @SuppressWarnings("rawtypes")
  protected void logValue(String label, Measure val) {
    logValue(label + " (" + val.baseUnit() + ")", (val != null ? val.baseUnitMagnitude() : 0));
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Additional abstract methods (besides those still abstract from IDrivebase)
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Allows the base class to access odometery data allocated specifically by a
   * derived type during construction. (Relies on hardware-specific elements that
   * are not available during AbstractDriveBase construction.)
   * 
   * @return odometry object for the drive base
   */
  protected abstract DifferentialDriveOdometry getOdometry();

  /**
   * Allows the base class to access pose estimator allocated specifically by a
   * derived type during construction. (Relies on hardware-specific elements that
   * are not available during AbstractDriveBase construction.)
   * 
   * @return pose estimator object for the drive base
   */
  protected abstract DifferentialDrivePoseEstimator getPoseEstimator();
}
