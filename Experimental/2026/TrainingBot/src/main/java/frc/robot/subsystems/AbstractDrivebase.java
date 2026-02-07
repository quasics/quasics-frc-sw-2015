// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for a drivebase.
 * 
 * This class provides a common interface for hardware-specific functionality,
 * and builds on that to provide higher-level functionality related to
 * driving/navigation.
 * 
 * Note that abstract classes can't be created directly; you need to create a
 * "derived" class that provides definitions for any methods marked as
 * "abstract" in this one, and then allocate/use that.
 * 
 * Partial example of this:
 * <code>
 *   // In AbstractDrivebase.java:
 *   public abstract class AbstractDrivebase {
 *     . . . .
 *     // Abstract method: anything that "extends" this type can be asked to do this.
 *     public abstract void setSpeeds(double leftPercent, double rightPercent);
 *     . . . .
 *   }
 * 
 *   // In RealDrivebase.java:
 *   public class RealDrivebase extends AbstractDrivebase {  // *Not* "abstract"
 *     . . . .
 *     // Non-abstract method, providing the implementation for a RealDrivebase
 *     // object.
 *     public void setSpeeds(double leftPercent, double rightPercent) {
 *       // Do some stuff
 *       . . . .
 *     }
 *     . . . .
 *   }
 * 
 *   // In RobotContainer.java:
 *   public class RobotContainer {
 *     . . . .
 *     AbstractDrivebase drivebase = new RealDrivebase(....);
 *     . . . .
 *   }
 * </code>
 * 
 * See the "DriveForDistance" command as an example of how an abstract type can
 * be used to define commands, etc., and then "plug in" the actual kind of
 * object that you want to work with.
 */
public abstract class AbstractDrivebase extends SubsystemBase {
  public static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase() {
    setName("Drivebase");
  }

  /////////////////////////////////////////////////////////////////////////////
  // Abstract (hardware-specific) methods that must be implemented by
  // subclasses.

  /** Sets the speeds for the left- and right-side motors. */
  public abstract void tankDrive(double leftPercentage, double rightPercentage);

  /**
   * @return the distance that the wheels on the left have traveled since startup
   *         (in meters).
   */
  public abstract double getLeftDistanceMeters();

  /**
   * @return the distance that the wheels on the right have traveled since
   *         startup (in meters).
   */
  public abstract double getRightDistanceMeters();

  /**
   * @return the robot's current heading in degrees, relative to the direction on
   *         startup.
   */
  public abstract double getHeadingInDegrees();

  /////////////////////////////////////////////////////////////////////////////
  // Driving methods that can be implemented using subclasses' methods.

  /**
   * Sets the speeds for the left- and right-side motors to the same percentage
   * (which should drive us straight forward or backward).
   */
  public void setSpeed(double percentage) {
    tankDrive(percentage, percentage);
  }

  /** Stops the drive base (setting motor speeds to 0). */
  public void stop() {
    tankDrive(0, 0);
  }

  /** @returns the robot's "pose" (for use with other WPILib functionality). */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Gets the distance that the robot's left wheel(s) have traveled since startup,
   * expressed as a "Distance" value (which automatically handles units).
   *
   * @return the distance that the wheels on the left have traveled since startup.
   *
   * @see #getLeftDistanceMeters()
   * @see edu.wpi.first.units.measure.Distance
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
   */
  public Distance getLeftDistance() {
    return Meters.of(getLeftDistanceMeters());
  }

  /**
   * Gets the distance that the robot's right wheel(s) have traveled since
   * startup, expressed as a "Distance" value (which automatically handles units).
   *
   * @return the distance that the wheels on the right have traveled since
   *         startup.
   *
   * @see #getRightDistanceMeters()
   * @see edu.wpi.first.units.measure.Distance
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
   */
  public Distance getRightDistance() {
    return Meters.of(getRightDistanceMeters());
  }

  /**
   * Gets the robot's current heading as an "Angle" value (which automatically
   * handles units).
   *
   * @return the robot's current heading
   *
   * @see #getHeadingInDegrees()
   * @see edu.wpi.first.units.measure.Angle
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
   */
  public Angle getHeading() {
    return Degrees.of(getHeadingInDegrees());
  }

  /////////////////////////////////////////////////////////////////////////////
  // Other functions

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Update our odometry
    final Rotation2d rotation = Rotation2d.fromDegrees(getHeadingInDegrees());
    final double leftDistanceMeters = getLeftDistanceMeters();
    final double rightDistanceMeters = getRightDistanceMeters();
    m_odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);

    SmartDashboard.putNumber("distance", leftDistanceMeters);
    SmartDashboard.putNumber("angle", getHeadingInDegrees());
  }
}
