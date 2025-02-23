// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basic interface for drive base functionality.
 */
public interface IDrivebase extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  final String SUBSYSTEM_NAME = "Drivebase";

  /** Key used to post Pose information to BulletinBoard. */
  final String POSE_KEY = SUBSYSTEM_NAME + ".Pose";

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.25);

  /** Maximum rotational velocity for arcade drive. */
  final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(.25 * Math.PI);

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
   */
  default void tankDrive(double percentage) {
    tankDrive(percentage, percentage);
  }

  /**
   * Drive the robot using tank drive (as a percentage of MAX_SPEED).
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
   * TODO: Consider rewriting this to use the "drive(ChassisSpeeds)" method.
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  default void arcadeDrive(LinearVelocity speed, AngularVelocity rotation) {
    // Calculate the left and right wheel speeds based on the inputs.
    final var wheelSpeeds = getKinematics().toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  /**
   * Set the wheel speeds (positive values are forward).
   *
   * @param wheelSpeeds The wheel speeds to set.
   */
  default void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // Calculate the left and right wheel speeds based on the inputs.
    final var leftSpeed = wheelSpeeds.leftMetersPerSecond;
    final var rightSpeed = wheelSpeeds.rightMetersPerSecond;

    // Set the speeds of the left and right sides of the drivetrain.
    final var maxSpeed = MAX_SPEED.in(MetersPerSecond);
    setMotorSpeeds(leftSpeed / maxSpeed, rightSpeed / maxSpeed);
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

  void setMotorVoltages(Voltage left, Voltage right);

  /**
   * Motor speed control (as a percentage).
   *
   * @param leftPercentage  left motor speed (as a percentage of full speed)
   * @param rightPercentage right motor speed (as a percentage of full speed)
   */
  void setMotorSpeeds(double leftPercentage, double rightPercentage);

  /** @return The applied voltage from the left motor */
  Voltage getLeftVoltage();

  /** @return The applied voltage from the right motor */
  Voltage getRightVoltage();

  /** @return The position reading from the left encoder */
  Distance getLeftPosition();

  /** @return The position reading from the right encoder */
  Distance getRightPosition();

  /** @return The velocity reading from the left encoder */
  LinearVelocity getLeftVelocity();

  /** @return The velocity reading from the right encoder */
  LinearVelocity getRightVelocity();

  /** @return the angular velocity of the robot (from the ALU) */
  AngularVelocity getTurnRate();

  /** @return heading of the robot (as an Angle) */
  Angle getHeading();

  /** @return heading of the robot, based on odometry */
  Pose2d getPose();

  Pose2d getEstimatedPose();

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
    drive(speeds);
  }

  default ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(getLeftVelocity(), getRightVelocity(), getTurnRate());
  }

  void resetPose(Pose2d pose);

  void drive(ChassisSpeeds speeds);

  LTVUnicycleController getLtvUnicycleController();
}
