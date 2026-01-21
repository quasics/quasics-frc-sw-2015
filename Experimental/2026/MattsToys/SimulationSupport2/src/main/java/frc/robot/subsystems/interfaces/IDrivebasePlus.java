// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.BulletinBoard;
import frc.robot.util.RobotConfigs.DriveConfig;

/** Interface for a drivebase subsystem with additional functionality. */
public interface IDrivebasePlus extends IDrivebase {
  //
  // Methods to return constants for the drivebase.
  //

  /**
   * @return the configuration data for the drivebase, including PID-oriented
   *         constants.
   */
  DriveConfig getConfig();

  default SimpleMotorFeedforward getFeedForward() {
    return new SimpleMotorFeedforward(
        getConfig().feedForward().linear().kS().in(Volts),
        getConfig().feedForward().linear().kV().in(Volts),
        getConfig().feedForward().linear().kA());
  }

  //
  // Methods to support more advanced control of the drivebase (e.g., profiling,
  // trajectory following, etc.).
  //

  /** Returns the (odometry-based) estimate of the robot's position. */
  public Pose2d getEstimatedPose();

  /**
   * Directly sets the voltages for the drivebase. (Note: operates directly; no
   * PID.)
   *
   * This is useful for motion profiling and other advanced control techniques.
   *
   * @param leftVoltage  voltage for the left side
   * @param rightVoltage voltage for the right side
   */
  public void tankDriveVolts(Voltage leftVoltage, Voltage rightVoltage);

  /**
   * Sets the speeds for the drivebase using chassis speeds. (Note: operates
   * directly; no PID.)
   *
   * @param speeds chassis speeds to set
   */
  void setSpeeds(ChassisSpeeds speeds);

  /**
   * Sets the speeds for the drivebase using PID control.
   *
   * @param speeds chassis speeds to set
   */
  void driveTankWithPID(ChassisSpeeds speeds);

  /** Returns the current position reported by the left encoder. */
  Distance getLeftPosition();

  /** Returns the current speed reported by the left encoder. */
  LinearVelocity getLeftVelocity();

  /**
   * Returns the current voltage reported by the left controller. (Useful for
   * profiling the robot.)
   */
  Voltage getLeftVoltage();

  /** Returns the current position reported by the right encoder. */
  Distance getRightPosition();

  /** Returns the current speed reported by the right encoder. */
  LinearVelocity getRightVelocity();

  /**
   * Returns the current voltage reported by the right controller. (Useful for
   * profiling the robot.)
   */
  Voltage getRightVoltage();

  /** Returns the current angular velocity of the robot. */
  AngularVelocity getAngularVelocity();

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Data published to BulletinBoard
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** Key used to post odometry-based pose information to BulletinBoard. */
  final String ODOMETRY_KEY = SUBSYSTEM_NAME + ".Pose";

  /**
   * Returns the latest posted odemetry-based pose (from the bulletin board).
   *
   * Note: this is a static function, which means that client code doesn't
   * interact directly with the actual subsystem (and thus doesn't need to
   * include it in their requirements).
   *
   * @return last posted odemetry pose, or null
   */
  static Pose2d getPublishedLastPoseFromOdometry() {
    var stored = BulletinBoard.common.getValue(ODOMETRY_KEY, Pose2d.class);
    return (Pose2d) stored.orElse(null);
  }

  public class NullDrivebase
      extends IDrivebase.NullDrivebase implements IDrivebasePlus {
    @Override
    public DriveConfig getConfig() {
      return null;
    }

    @Override
    public Pose2d getEstimatedPose() {
      return new Pose2d();
    }

    @Override
    public void tankDriveVolts(Voltage leftVoltage, Voltage rightVoltage) {
      // No-op
    }

    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
      // No-op
    }

    @Override
    public void driveTankWithPID(ChassisSpeeds speeds) {
      // No-op
    }

    @Override
    public Distance getLeftPosition() {
      return Meters.of(0);
    }

    @Override
    public LinearVelocity getLeftVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public Voltage getLeftVoltage() {
      return Volts.of(0);
    }

    @Override
    public Distance getRightPosition() {
      return Meters.of(0);
    }

    @Override
    public LinearVelocity getRightVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public Voltage getRightVoltage() {
      return Volts.of(0);
    }

    @Override
    public AngularVelocity getAngularVelocity() {
      return RadiansPerSecond.of(0);
    }
  }
}
