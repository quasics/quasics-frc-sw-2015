package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro; // Example Gyro
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.ModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
  // 1. Module Instances
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  // 2. Kinematics/Odometry Models
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  // 3. Sensor (Gyroscope)
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public SwerveDriveSubsystem() {
    frontLeft = new SwerveModule(
        ModuleConstants.kFrontLeftDriveCANId, ModuleConstants.kFrontLeftSteerCANId,
        ModuleConstants.kFrontLeftOffset);
    frontRight = new SwerveModule(
        ModuleConstants.kFrontRightDriveCANId, ModuleConstants.kFrontRightSteerCANId,
        ModuleConstants.kFrontRightOffset);

    rearLeft = new SwerveModule(
        ModuleConstants.kRearLeftDriveCANId, ModuleConstants.kRearLeftSteerCANId,
        ModuleConstants.kRearLeftOffset);
    rearRight = new SwerveModule(
        ModuleConstants.kRearRightDriveCANId, ModuleConstants.kRearRightSteerCANId,
        ModuleConstants.kRearRightOffset);

    // Define Module Locations (Kinematics)
    // These are the (x, y) coordinates of the center of each module
    // relative to the robot's center (in meters)
    Translation2d frontLeftLocation = new Translation2d(
        DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);
    Translation2d frontRightLocation = new Translation2d(
        -1 * DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);
    Translation2d rearLeftLocation = new Translation2d(
        DriveConstants.kTrackWidth / 2.0, -1 * DriveConstants.kWheelBase / 2.0);
    Translation2d rearRightLocation = new Translation2d(
        -1 * DriveConstants.kTrackWidth / 2.0, -1 * DriveConstants.kWheelBase / 2.0);

    // Instantiate the Kinematics object
    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

    // Instantiate Odometry (Requires Gyro Angle and Initial Pose)
    odometry = new SwerveDriveOdometry(
        kinematics, // Kinematics
        Rotation2d.fromDegrees(gyro.getAngle()), // Current gyro heading
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
  }

  /**
   * Controls the swerve drive.
   * 
   * @param xSpeed        The desired velocity along the X axis (forward/backward)
   *                      in m/s.
   * @param ySpeed        The desired velocity along the Y axis (strafe) in m/s.
   * @param rot           The desired angular rate of the robot in rad/s.
   * @param fieldRelative Whether the speeds are relative to the field or the
   *                      robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Create a ChassisSpeeds object based on driver input
    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      // Field-Centric Control: Gyro angle is used for translation
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, rot, getGyroscopeRotation());
    } else {
      // Robot-Centric Control: Simple, but generally less intuitive
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    // Convert the desired ChassisSpeeds into individual SwerveModuleStates
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Normalize the wheel speeds to ensure none exceed the max speed limit
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Send the final calculated states to each module's setDesiredState() method
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    rearLeft.setDesiredState(moduleStates[2]);
    rearRight.setDesiredState(moduleStates[3]);
  }

  /** Utility method to stop the drive base. */
  public void stop() {
    drive(0, 0, 0, true);
  }

  @Override
  public void periodic() {
    // Update the odometry every cycle (crucial for localization)
    odometry.update(
        getGyroscopeRotation(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
  }

  /** Returns the rotation of the robot from the Gyroscope. */
  public Rotation2d getGyroscopeRotation() {
    // Return the negative of the gyro angle if positive is clockwise
    // (WPILib standard is Counter-Clockwise positive)
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /** Resets the Gyroscope angle and Odometry pose to zero. */
  public void resetDriveEncodersAndGyro() {
    gyro.reset();
    odometry.resetPosition(
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition() },
        new Pose2d());
  }
}
