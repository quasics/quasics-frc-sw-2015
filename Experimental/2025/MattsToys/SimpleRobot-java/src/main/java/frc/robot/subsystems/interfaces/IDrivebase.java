// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

/**
 * Basic interface for relatively simple drive base functionality, including tank/arcade drive,
 * reading key sensors (encoders/ALU).
 *
 * Note that I'm currently breaking this out into:
 * <ul>
 * <li>A simple interface (this one) for truly core functionality (and default implementations of
 * simple things).
 * <li>A derived interface (IBetterDrivebase), which would be used to help define more advanced
 * functionality.
 * <li>An abstract class, which starts handling things like PID, etc.
 * <li>Concrete types, which mostly serve to set up/access the underlying hardware (real or
 * simulated).
 * </ul>
 *
 * Possible enhancements:
 * <ul>
 * <li>
 * Use either the raw odometry or (unified) pose estimation to provide a signal
 * to the drive team about the robot's position on the field (e.g., when it's
 * oriented towards the barge and close enough to make the shot). This could be
 * done by putting something on the dashboard, changing the lights on the robot,
 * etc.  (Note: a simple implementation of this type of functionality may be
 * seen in the <code>DriveTeamShootingSupport</code> command.)
 * </li>
 * <li>
 * Use odemetry/pose estimation to automatically move the robot to the correct
 * position/orientation for shooting algae into the barge.
 * </li>
 * </ul>
 */
public interface IDrivebase extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  final String SUBSYSTEM_NAME = "Drivebase";

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  final AngularVelocity MAX_ROTATION = DegreesPerSecond.of(180);

  /** Zero linear velocity. (A potentially useful constant.) */
  final LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity.  (A potentially useful constant.) */
  final AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /** Utility method: stops the robot. */
  default void stop() {
    tankDrive(0, 0);
  }

  /**
   * Utility method: straight forward/backward. (Effectively, tank drive with a
   * single speed for both sides.)
   *
   * @param percentage The percentage of MAX_SPEED to drive at (positive is forward).
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
   * @param leftPercentage  The percentage of MAX_SPEED for the left side (positive is forward).
   * @param rightPercentage The percentage of MAX_SPEED for the right side (positive is forward).
   */
  default void tankDrive(double leftPercentage, double rightPercentage) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedLeftPercentage = MathUtil.clamp(leftPercentage, -1.0, +1.0);
    double clampedRightPercentage = MathUtil.clamp(rightPercentage, -1.0, +1.0);

    setSpeeds(new DifferentialDriveWheelSpeeds(
        MAX_SPEED.times(clampedLeftPercentage), MAX_SPEED.times(clampedRightPercentage)));
  }

  /**
   * Drive the robot using arcade drive.
   *
   * Note: operates directly; no PID, but clamped to MAX_SPEED.
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  default void arcadeDrive(LinearVelocity speed, AngularVelocity rotation) {
    if (speed == null) {
      speed = ZERO_MPS;
    }
    if (rotation == null) {
      rotation = ZERO_TURNING;
    }

    // Calculate the left and right wheel speeds based on the inputs.
    final DifferentialDriveWheelSpeeds wheelSpeeds =
        getKinematics().toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  /**
   * Set the wheel speeds (positive values are forward).
   *
   * Note: operates directly; no PID, but clamped to MAX_SPEED.
   *
   * @param wheelSpeeds The wheel speeds to set.
   *
   * @see #setMotorSpeeds
   */
  default void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // Calculate the (clamped) left and right wheel speeds based on the inputs.
    final double leftSpeed = MathUtil.clamp(wheelSpeeds.leftMetersPerSecond,
        -MAX_SPEED.in(MetersPerSecond), MAX_SPEED.in(MetersPerSecond));
    final double rightSpeed = MathUtil.clamp(wheelSpeeds.rightMetersPerSecond,
        -MAX_SPEED.in(MetersPerSecond), MAX_SPEED.in(MetersPerSecond));

    // Set the speeds of the left and right sides of the drivetrain.
    final var maxSpeed = MAX_SPEED.in(MetersPerSecond);
    setMotorSpeeds(leftSpeed / maxSpeed, rightSpeed / maxSpeed);
  }

  /**
   * Motor speed control (as a percentage).
   *
   * Note: operates directly; no PID, and does not reference MAX_SPEED, though
   * percentages are clamped to [-1,+1].
   *
   * @param leftPercentage  left motor speed (as a percentage of full speed)
   * @param rightPercentage right motor speed (as a percentage of full speed)
   */
  default void setMotorSpeeds(double leftPercentage, double rightPercentage) {
    final double referenceVoltage = RobotController.getInputVoltage();
    setMotorVoltages(Volts.of(referenceVoltage * MathUtil.clamp(leftPercentage, -1, +1)),
        Volts.of(referenceVoltage * MathUtil.clamp(rightPercentage, -1, +1)));
  }

  /**
   * Returns the left encoder's position.
   *
   * @return The position reading from the left encoder
   */
  default Distance getLeftPosition() {
    return getLeftEncoder().getPosition();
  }

  /**
   * Returns the right encoder's position.
   *
   * @return The position reading from the right encoder
   */
  default Distance getRightPosition() {
    return getRightEncoder().getPosition();
  }

  /**
   * Returns the left encoder's velocity.
   *
   * @return The position reading from the left velocity
   */
  default LinearVelocity getLeftVelocity() {
    return getLeftEncoder().getVelocity();
  }

  /**
   * Returns the right encoder's velocity.
   *
   * @return The position reading from the right velocity
   */
  default LinearVelocity getRightVelocity() {
    return getRightEncoder().getVelocity();
  }

  /**
   * Returns the robot's angular velocity (from the ALU).
   *
   * @return the angular velocity of the robot
   */
  default AngularVelocity getTurnRate() {
    return getGyro().getRate();
  }

  /**
   * Returns the robot's heading (from the ALU).
   *
   * @return the heading of the robot
   */
  default Angle getHeading() {
    return getGyro().getAngle();
  }

  /**
   * Gets the robot's current speeds (wheels and turning).
   *
   * @return the current ChassisSpeeds of the robot (used for
   *         trajectory-following)
   */
  default ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(getLeftVelocity(), getRightVelocity(), getTurnRate());
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
   * Returns the voltage being applied to the left motor.
   *
   * @return The applied voltage from the left motor
   */
  Voltage getLeftVoltage();

  /**
   * Returns the voltage being applied to the right motor.
   *
   * @return The applied voltage from the right motor
   */
  Voltage getRightVoltage();

  /**
   * Returns a TrivialEncoder for the left motor.
   *
   * @return TrivialEncoder exposing data for the left motors
   */
  TrivialEncoder getLeftEncoder();

  /**
   * Returns a TrivialEncoder for the right motor.
   *
   * @return TrivialEncoder exposing data for the right motors
   */
  TrivialEncoder getRightEncoder();

  /**
   * Exposes a gyro (providing heading/yaw) for the robot.
   *
   * @return IGyro exposing data from the underlying ALU
   */
  IGyro getGyro();

  /**
   * Gets the robot's kinematics.
   *
   * @return kinematics data for the robot
   */
  DifferentialDriveKinematics getKinematics();

  /** Trivial implementation of the IDrivebase interface. */
  public static class NullDrivebase implements IDrivebase {
    /** No-op encoder. */
    static final TrivialEncoder NULL_ENCODER = new TrivialEncoder.NullEncoder();
    /** No-op gyro. */
    static final IGyro NULL_GYRO = new IGyro.NullGyro();

    /** Constructor. */
    public NullDrivebase() {
      System.out.println("INFO: Allocating null drivebase");
    }

    @Override
    public void setMotorVoltages(Voltage left, Voltage right) {
      // No-op
    }

    @Override
    public Voltage getLeftVoltage() {
      return Volts.of(0);
    }

    @Override
    public Voltage getRightVoltage() {
      return Volts.of(0);
    }

    @Override
    public TrivialEncoder getLeftEncoder() {
      return NULL_ENCODER;
    }

    @Override
    public TrivialEncoder getRightEncoder() {
      return NULL_ENCODER;
    }

    @Override
    public IGyro getGyro() {
      return NULL_GYRO;
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
      return new DifferentialDriveKinematics(Meters.of(0.5));
    }
  }
}
