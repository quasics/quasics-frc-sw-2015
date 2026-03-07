// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.function.Supplier;

public interface IDrivebase {
  /**
   * Drives the robot using arcade controls. (That is, one parameter controls
   * forward/backward speed, and the other controls turning speed.)
   *
   * The implementation is expected to combine these in a way that allows the
   * robot to do both at the same time, and to turn in place when the forward
   * speed is zero.
   *
   * @param forwardspeed forward/backward speed (positive is forward, negative
   *                     is backward)
   * @param turnspeed    turning speed (positive is clockwise, negative is
   *                     counterclockwise)
   */
  void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  /**
   * Drives the robot using "tank drive" style controls. (That is, one parameter
   * controls the speed of the left side of the drivebase, and the other
   * controls the speed of the right side.)
   *
   * @param leftSpeed  the speed for the left side of the drivebase (positive is
   *                   forward, negative is backward)
   * @param rightSpeed the speed for the right side of the drivebase (positive
   *                   is
   *                   forward, negative is backward)
   */
  void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed);

  /**
   * Drives the robot using "tank drive" style controls. (That is, one parameter
   * controls the speed of the left side of the drivebase, and the other
   * controls the speed of the right side.)
   *
   * The parameters are expected to be in the range [-1, 1], where 1 represents
   * full forward speed, and -1 represents full backward speed. (The
   * implementation should ensure that values outside this range are handled
   * appropriately, e.g., by clamping them to the range.)
   *
   * @param leftPercent  the percentage of full speed for the left side of the
   *                     drivebase
   * @param rightPercent the percentage of full speed for the right side of the
   *                     drivebase
   */
  void setPercent(double leftPercent, double rightPercent);

  /**
   * Stops the robot by setting the motor speeds to zero. (This is a convenience
   * method that can be called from commands, etc., to stop the robot without
   * having to know the details of how to set the speeds to zero.)
   */
  default void stop() {
    setSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0));
  }

  // Used to set voltage directly to the motors (for characterization,
  // trajectory following, etc.)
  void setVoltages(Voltage leftVoltage, Voltage rightVoltage);

  double mpsToPercent(LinearVelocity speed);

  double getHeading();

  AngularVelocity getTurnRate();

  Pose2d getOdometryPose();

  Pose2d getEstimatedPose();

  void resetOdometry(Pose2d pose);

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);

  Command sysIdQuasistatic(IDrivebase drivebase, Mode mode, SysIdRoutine.Direction direction);

  Command sysIdDynamic(IDrivebase drivebase, Mode mode, SysIdRoutine.Direction direction);

  Distance getLeftDistance();

  Distance getRightDistance();

  double getLeftRawDistance();

  double getRightRawDistance();

  LinearVelocity getLeftVelocity();

  LinearVelocity getRightVelocity();

  /**
   * Tries to enable/disable breaking mode, if supported by the underlying
   * hardware.
   * 
   * @param enable if true, breaking mode should be enabled; if false, "coast
   *               mode" should be enabled
   * @return true if we were able to apply the mode to the hardware; false if the
   *         operation isn't supported for this drivebase
   */
  boolean setBreakingMode(boolean enable);

  //////////////////////////////////////////////
  // Auto utilities
  ChassisSpeeds getSpeed();

  void setSpeed(ChassisSpeeds speed);

  //////////////////////////////////////////////

  public enum Mode {
    Linear,
    Angular
  }

  /**
   * Trivial implementation of IDrivebase, for use when we're on a robot without
   * one. (As if this will ever happen?)
   */
  public class NullDrivebase extends SubsystemBase implements IDrivebase {
    public NullDrivebase() {
    }

    @Override
    public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed) {
      // No-op.
    }

    @Override
    public void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed) {
      // No-op.
    }

    @Override
    public void setPercent(double leftPercent, double rightPercent) {
      // No-op.
    }

    @Override
    public void setVoltages(Voltage leftVoltage, Voltage rightVoltage) {
      // No-op.
    }

    @Override
    public double mpsToPercent(LinearVelocity speed) {
      return 0;
    }

    @Override
    public double getHeading() {
      return new Rotation2d().getDegrees();
    }

    @Override
    public AngularVelocity getTurnRate() {
      return RadiansPerSecond.of(0);
    }

    @Override
    public Pose2d getOdometryPose() {
      return new Pose2d();
    }

    @Override
    public Pose2d getEstimatedPose() {
      return new Pose2d();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
      // No-op.
    }

    @Override
    public void setReferencePositionSupplier(Supplier<Pose2d> supplier) {
      // No-op.
    }

    @Override
    public Command sysIdQuasistatic(IDrivebase drivebase, Mode mode, Direction direction) {
      return Commands.print("Can't perform characterization on a NullDrivebase");
    }

    @Override
    public Command sysIdDynamic(IDrivebase drivebase, Mode mode, Direction direction) {
      return Commands.print("Can't perform characterization on a NullDrivebase");
    }

    @Override
    public Distance getLeftDistance() {
      return Meters.of(0);
    }

    @Override
    public Distance getRightDistance() {
      return Meters.of(0);
    }

    @Override
    public LinearVelocity getLeftVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public LinearVelocity getRightVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public double getLeftRawDistance() {
      return 0;
    }

    @Override
    public double getRightRawDistance() {
      return 0;
    }

    @Override
    public boolean setBreakingMode(boolean enable) {
      return false; // Not supported
    }

    @Override
    public ChassisSpeeds getSpeed() {
      return new ChassisSpeeds();
    }

    @Override
    public void setSpeed(ChassisSpeeds speed) {
      // Intentionally empty
    }
  }
}
