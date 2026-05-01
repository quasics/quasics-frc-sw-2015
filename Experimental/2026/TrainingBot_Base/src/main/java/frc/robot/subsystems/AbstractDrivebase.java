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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for a drivebase.
 * 
 * This class provides a common interface for hardware-specific functionality,
 * and builds on that to provide higher-level functionality related to
 * driving/navigation.
 * 
 * Note that this is declared to know about "MotorController" (which is an
 * interface) objects when defining the variables for the left and right motor
 * controllers; these can be assigned any object that implements the
 * MotorController interface (e.g., a TalonSRX, a Spark, a PWMVictorSPX, etc.).
 * This allows us to use the same base class for different kinds of *actual*
 * drive base hardware, even though they will be using different motor
 * controller objects under the hood. It will be the job of the "derived"
 * classes to create the actual motor controller objects and pass them to the
 * base class, which will then use them to implement the higher-level
 * functionality (e.g., tank drive, arcade drive, etc.) that we want to use in
 * our commands.
 * 
 * Note also that abstract classes can't be created directly; you need to create
 * a derived class that provides definitions for any methods marked as
 * "abstract" in this one, and then allocate/use that.
 * 
 * Partial example of this:
 * <code>
 *   // In AbstractDrivebase.java:
 *   public abstract class AbstractDrivebase {
 *     . . . .
 *     // Abstract method: anything that "extends" this type can be asked to do this.
 *     public abstract double getHeadingInDegrees();
 *     . . . .
 *   }
 * 
 *   // In RealDrivebase.java:
 *   public class RealDrivebase extends AbstractDrivebase {  // *Not* "abstract"
 *     . . . .
 *     // Non-abstract method, providing the implementation for a RealDrivebase
 *     // object.
 *     public double getHeadingInDegrees() {
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
  /** A constant representing zero speed (in meters per second). */
  public static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  /** The motor controller for the left side of the drivebase. */
  final protected MotorController m_leftController;

  /** The motor controller for the right side of the drivebase. */
  final protected MotorController m_rightController;

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  /**
   * A WPILib "DifferentialDrive" object, which provides some built-in driving
   * functionality (e.g., tank drive, arcade drive, etc.) that we can use
   * directly in our class, without having to re-implement that logic ourselves.
   * 
   * While this does make some things simpler for us, it also means that we will
   * have to make that we "feed" the differential drive object new valuaes for the
   * left and right motor speeds on a regular basis (e.g., in the "periodic"
   * method of this class, or in the execute() method of any command that uses
   * this subsystem); otherwise, the differential drive object will start
   * reporting errors about not being "fed" often enough, and it may stop working
   * correctly (i.e., turn off the motors, rather than continuing at the last-set
   * speeds).
   * 
   * @see edu.wpi.first.wpilibj.MotorSafety
   */
  final protected DifferentialDrive m_differentialDrive;

  /**
   * Constructor.
   * 
   * @param leftController  the motor controller for the left side of the
   *                        drivebase (provided by the derived class)
   * @param rightController the motor controller for the right side of the
   *                        drivebase (provided by the derived class)
   */
  public AbstractDrivebase(MotorController leftController, MotorController rightController) {
    // Give the subsystem a name (this is optional, but can be helpful for debugging
    // and dashboard purposes).
    setName("Drivebase");

    // Store the motor controllers, and create the differential drive object that
    // will use them.
    m_leftController = leftController;
    m_rightController = rightController;
    m_differentialDrive = new DifferentialDrive(m_leftController, m_rightController);
  }

  /** Sets the speeds for the left- and right-side motors (tank drive). */
  public void tankDrive(double leftPercentage, double rightPercentage) {
    // Clamp speeds to the range [-1.0, 1.0].
    leftPercentage = Math.max(-1.0, Math.min(1.0, leftPercentage));
    rightPercentage = Math.max(-1.0, Math.min(1.0, rightPercentage));

    m_differentialDrive.tankDrive(leftPercentage, rightPercentage);
  }

  /** Sets the speeds for the linear and rotational motion (arcade drive). */
  public void arcadeDrive(double speed, double rotation) {
    // Clamp speeds to the range [-1.0, 1.0].
    speed = Math.max(-1.0, Math.min(1.0, speed));
    rotation = Math.max(-1.0, Math.min(1.0, rotation));

    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Abstract (hardware-specific) methods that must be implemented by
  // subclasses.

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
  // Driving methods that can be implemented using subclasses' methods, or
  // those defined in this class.

  /**
   * Sets the speeds for the left- and right-side motors to the same percentage
   * (which should drive us straight forward or backward).
   * 
   * @param percentage percentage of drive speed (-1.0 to +1.0)
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
   * Note that this really is a safer way to handle distances, since it will
   * automatically convert between different units if needed, and it will also
   * make it harder to accidentally mix up different units (e.g., by accidentally
   * using meters in one place and inches in another, which can lead to bugs that
   * are hard to track down). However, it does require a bit more code to use,
   * since you have to call the "of" method to create a Distance object from a raw
   * value, and then call the appropriate method to get the value back out in the
   * units you want (e.g., getMeters(), getInches(), etc.). So, it's up to you
   * whether you want to use this approach or just use raw double values for
   * distances.
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

    // Display some values on the dashboard for debugging purposes.
    SmartDashboard.putNumber("L distance (m)", leftDistanceMeters);
    SmartDashboard.putNumber("R distance (m)", rightDistanceMeters);
    SmartDashboard.putNumber("Heading (deg)", getHeadingInDegrees());
  }
}
