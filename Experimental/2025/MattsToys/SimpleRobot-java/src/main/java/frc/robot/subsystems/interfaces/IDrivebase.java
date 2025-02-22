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
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  void arcadeDrive(LinearVelocity speed, AngularVelocity rotation);

  /**
   * Set the wheel speeds (positive values are forward).
   *
   * @param wheelSpeeds The wheel speeds to set.
   */
  void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds);

  /**
   * Utility method: straight forward/backward. (Effectively, tank drive with a
   * single speed for both sides.)
   *
   * @param percentage The percentage of MAX_SPEED to drive at.
   */
  default void setSpeed(double percentage) {
    tankDrive(percentage, percentage);
  }

  /** Utility method: stops the robot. */
  default void stop() {
    tankDrive(0, 0);
  }

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

  /////////////////////////////////////////////////////////////////////////////////
  //
  // "Purely abstract methods"
  //
  /////////////////////////////////////////////////////////////////////////////////

  void setMotorVoltages(Voltage left, Voltage right);

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

  /** @return heading of the robot (as an Angle) */
  Angle getHeading();

  /** @return heading of the robot, based on odometry */
  Pose2d getPose();

  Pose2d getEstimatedPose();

  //
  // Functionality required for AutoBuilder (in PathPlanner library) or
  // AutoFactory (in Choreo library)
  //

  void resetPose(Pose2d pose);

  ChassisSpeeds getCurrentSpeeds();

  void drive(ChassisSpeeds speeds);

  LTVUnicycleController getLtvUnicycleController();
}
