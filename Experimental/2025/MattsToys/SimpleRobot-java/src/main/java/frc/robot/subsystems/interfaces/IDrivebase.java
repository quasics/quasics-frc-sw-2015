// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.BulletinBoard;

/**
 * Basic interface for drive base functionality.
 * 
 * TODO: At this point, this *really* ought to be an abstract class, vs an
 * interface, based on the amount of code it contains.
 */
public interface IDrivebase extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  final String SUBSYSTEM_NAME = "Drivebase";

  /** Key used to post odometry-based pose information to BulletinBoard. */
  final String POSE_KEY = SUBSYSTEM_NAME + ".Pose";

  /** Key used to post odometry-based pose information to BulletinBoard. */
  final String ESTIMATED_POSE_KEY = SUBSYSTEM_NAME + ".PoseEstimate";

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  final AngularVelocity MAX_ROTATION = DegreesPerSecond.of(180);

  /** Zero velocity. (A potentially useful constant.) */
  final LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  final boolean LOG_TO_SMARTDASHBOARD = true;

  /** Utility method: stops the robot. */
  default void stop() {
    tankDrive(0, 0);
  }

  /**
   * Utility method: straight forward/backward. (Effectively, tank drive with a
   * single speed for both sides.)
   *
   * @param percentage The percentage of MAX_SPEED to drive at.
   * 
   * @see #tankDrive(double, double)
   */
  default void tankDrive(double percentage) {
    tankDrive(percentage, percentage);
  }

  /**
   * Drive the robot using tank drive (as a percentage of MAX_SPEED).
   * 
   * Note: operates directly; no PID, but based on MAX_SPEED.
   *
   * @param leftPercentage  The percentage of MAX_SPEED for the left side.
   * @param rightPercentage The percentage of MAX_SPEED for the right side.
   */
  default void tankDrive(double leftPercentage, double rightPercentage) {
    setSpeeds(new DifferentialDriveWheelSpeeds(
        MAX_SPEED.times(leftPercentage), MAX_SPEED.times(rightPercentage)));
  }

  /**
   * Drive the robot using arcade drive.
   * 
   * Note: operates directly; no PID, but clamped to MAX_SPEED.
   * 
   * TODO: Consider rewriting this to use the "drive(ChassisSpeeds)" method.
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  default void arcadeDrive(LinearVelocity speed, AngularVelocity rotation) {
    // Calculate the left and right wheel speeds based on the inputs.
    final DifferentialDriveWheelSpeeds wheelSpeeds = getKinematics()
        .toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  /**
   * Set the wheel speeds (positive values are forward).
   * 
   * Note: operates directly; no PIDm but clamped to MAX_SPEED.
   *
   * @param wheelSpeeds The wheel speeds to set.
   */
  default void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // Calculate the (clamped) left and right wheel speeds based on the inputs.
    final double leftSpeed = MathUtil.clamp(
        wheelSpeeds.leftMetersPerSecond,
        -MAX_SPEED.in(MetersPerSecond),
        MAX_SPEED.in(MetersPerSecond));
    final double rightSpeed = MathUtil.clamp(
        wheelSpeeds.rightMetersPerSecond,
        -MAX_SPEED.in(MetersPerSecond),
        MAX_SPEED.in(MetersPerSecond));

    // Set the speeds of the left and right sides of the drivetrain.
    final var maxSpeed = MAX_SPEED.in(MetersPerSecond);
    setMotorSpeeds(leftSpeed / maxSpeed, rightSpeed / maxSpeed);
  }

  /**
   * Update the odometry/pose estimation, based on current sensor data.
   */
  default void updateOdometry(DifferentialDriveOdometry odometry, DifferentialDrivePoseEstimator estimator) {
    final Rotation2d rotation = getGyro().getRotation2d();
    final double leftDistanceMeters = getLeftEncoder().getPosition().in(Meters);
    final double rightDistanceMeters = getRightEncoder().getPosition().in(Meters);

    if (odometry != null) {
      odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
    }

    if (estimator != null) {
      estimator.update(rotation, leftDistanceMeters, rightDistanceMeters);

      // If an estimated position has been posted by the vision subsystem, integrate
      // it into our estimate.
      Optional<Object> optionalPose = BulletinBoard.common.getValue(IVision.VISION_POSE_KEY, Pose2d.class);
      optionalPose.ifPresent(poseObject -> {
        BulletinBoard.common.getValue(IVision.VISION_TIMESTAMP_KEY, Double.class)
            .ifPresent(timestampObject -> estimator.addVisionMeasurement(
                (Pose2d) poseObject, (Double) timestampObject));
      });
    }
  }

  /** Share the current pose with other subsystems (e.g., vision). */
  default void publishData(PIDController leftPidController, PIDController rightPidController) {
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
    if (leftPidController != null) {
      SmartDashboard.putData("Drive pid (L)", leftPidController);
    }
    if (rightPidController != null) {
      SmartDashboard.putData("Drive pid (R)", rightPidController);
    }
  }

  /** @return The position reading from the left encoder */
  default Distance getLeftPosition() {
    return getLeftEncoder().getPosition();
  }

  /** @return The position reading from the right encoder */
  default Distance getRightPosition() {
    return getRightEncoder().getPosition();
  }

  /** @return The velocity reading from the left encoder */
  default LinearVelocity getLeftVelocity() {
    return getLeftEncoder().getVelocity();
  }

  /** @return The velocity reading from the right encoder */
  default LinearVelocity getRightVelocity() {
    return getLeftEncoder().getVelocity();
  }

  /** @return the angular velocity of the robot (from the ALU) */
  default AngularVelocity getTurnRate() {
    return getGyro().getRate();
  }

  /** @return heading of the robot (as an Angle) */
  default Angle getHeading() {
    return getGyro().getAngle();
  }

  /**
   * @return the current ChassisSpeeds of the robot (used for
   *         trajectory-following)
   */
  default ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(getLeftVelocity(), getRightVelocity(), getTurnRate());
  }

  /**
   * Sets the speeds for the robot.
   * 
   * Used for trajectory-following (e.g., with Choreo).
   * 
   * @param speeds desired left/right/rotational speeds
   * 
   * @see #driveWithPid(DifferentialDriveWheelSpeeds)
   */
  default void driveWithPid(ChassisSpeeds speeds) {
    driveWithPid(getKinematics().toWheelSpeeds(speeds));
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Utility logging methods
  //
  /////////////////////////////////////////////////////////////////////////////////

  @SuppressWarnings("rawtypes")
  default void logValue(String label, Measure val) {
    logValue(
        label + " (" + val.baseUnit() + ")",
        (val != null ? val.baseUnitMagnitude() : 0));
  }

  default void logValue(String label, double val) {
    if (LOG_TO_SMARTDASHBOARD) {
      SmartDashboard.putNumber(label, val);
    }
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // "Purely abstract methods"
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Directly sets the voltages delivered to the motors.
   * 
   * Note: operates directly; no PID.
   * 
   * @param left  voltage for the left-side motors
   * @param right voltage for the right-side motors
   */
  void setMotorVoltages(Voltage left, Voltage right);

  /**
   * Motor speed control (as a percentage).
   * 
   * Note: operates directly; no PID, nut does not reference MAX_SPEED.
   * 
   * TODO: Add a default implementation based on setMotorVoltages().
   *
   * @param leftPercentage  left motor speed (as a percentage of full speed)
   * @param rightPercentage right motor speed (as a percentage of full speed)
   */
  void setMotorSpeeds(double leftPercentage, double rightPercentage);

  /** @return The applied voltage from the left motor */
  Voltage getLeftVoltage();

  /** @return The applied voltage from the right motor */
  Voltage getRightVoltage();

  /** @return TrivialEncoder exposing data for the left motors */
  TrivialEncoder getLeftEncoder();

  /** @return TrivialEncoder exposing data for the right motors */
  TrivialEncoder getRightEncoder();

  /** @return IGyro exposing data from the underlying ALU */
  IGyro getGyro();

  /** @return position/heading of the robot, based on odometry */
  Pose2d getPose();

  /** @return position/heading of the robot, based on pose estimation */
  Pose2d getEstimatedPose();

  /** @return kinematics data for the robot */
  DifferentialDriveKinematics getKinematics();

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Functionality required for AutoBuilder (in PathPlanner library) or
  // AutoFactory (in Choreo library)
  //
  // See: https://choreo.autos/choreolib/getting-started/
  // See: https://www.chiefdelphi.com/t/choreo-2025-beta/472224/23
  // See: https://github.com/mjansen4857/pathplanner/tree/main/examples/java
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Follows (executes) a sample of a Choreo trajectory for a differential drive
   * robot.
   * 
   * @param sample component of the trajectory to be followed
   * 
   * @see #driveWithPid(ChassisSpeeds)
   */
  default void followTrajectory(DifferentialSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Get the velocity feedforward specified by the sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = getLtvUnicycleController().calculate(
        pose,
        sample.getPose(),
        ff.vxMetersPerSecond,
        ff.omegaRadiansPerSecond);

    // Apply the generated speeds
    driveWithPid(speeds);
  }

  void resetPose(Pose2d pose);

  public void driveWithPid(DifferentialDriveWheelSpeeds wheelSpeeds);

  LTVUnicycleController getLtvUnicycleController();
}
