// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  /////////////////////////////////////////////////////////////////////////////////
  //
  // SysId (profiling) support
  //
  /////////////////////////////////////////////////////////////////////////////////

  boolean DUMP_SYSID_TO_CONSOLE = true;

  // Create a new SysId routine for characterizing the drive.
  default SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step
        // voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
              setMotorVoltages(volts, volts);
            },
            // Tell SysId how to record a frame of data for each motor on the
            // mechanism being characterized.
            log -> {
              final var leftPosition = getLeftPosition();
              final var leftVelocity = getLeftVelocity();
              final var leftVoltage = getLeftVoltage();
              final var rightPosition = getRightPosition();
              final var rightVelocity = getRightVelocity();
              final var rightVoltage = getRightVoltage();

              if (DUMP_SYSID_TO_CONSOLE) {
                System.err.println("Logging "
                    + "left=" + String.format("%,.3f", leftVoltage) + "V, "
                    + String.format("%,.3f", leftPosition.in(Meters)) + "m, "
                    + String.format("%,.3f", leftVelocity.in(MetersPerSecond)) + "m/s   "
                    + "right=" + String.format("%,.3f", rightVoltage) + "V, "
                    + String.format("%,.3f", rightPosition.in(Meters)) + "m, "
                    + String.format("%,.3f", rightVelocity.in(MetersPerSecond)) + "m/s   ");
              }

              // Record a frame for the left motors. Since these share an encoder,
              // we consider the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(leftVoltage)
                  .linearPosition(leftPosition)
                  .linearVelocity(leftVelocity);
              // Record a frame for the right motors. Since these share an
              // encoder, we consider the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(rightVoltage)
                  .linearPosition(rightPosition)
                  .linearVelocity(rightVelocity);
            },
            // Tell SysId to make generated commands require this subsystem,
            // suffix test state in WPILog with this subsystem's name (e.g., "drive")
            asSubsystem()));
  }

  /**
   * @return a Command for use in running quasistatic profiling in the
   *         specified direction.
   */
  default Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().quasistatic(direction);
  }

  /**
   * @return a Command for use in running dynamic profiling in the
   *         specified direction.
   */
  default Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().dynamic(direction);
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

  /** @return heading of the robot (as a Pose2d) */
  Pose2d getPose();
}
