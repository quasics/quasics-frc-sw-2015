// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public abstract class AbstractDrivebase extends SubsystemBase {
  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3);

  // Max rotational speed is 1/2 rotations per second
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(8.42);

  protected static final double NEO_FREE_SPEED = 5676;

  private static final boolean ENABLE_VOLTAGE_APPLICATON = true;

  protected static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final Distance m_driveBaseLengthWithBumpers;
  private final Distance m_driveBaseWidthWithBumpers;

  RobotConfig config;

  /** Creates a new IDrivebase. */
  public AbstractDrivebase(RobotSettings.Robot robot) {
    this(robot.trackWidthMeters);
    super.setName(robot.name());

    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                               // starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> setSpeeds(
              speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                       // Also optionally outputs individual module feedforwards
          new PPLTVController(0.02), // PPLTVController is the built in path following controller
                                     // for differential drive trains
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Blue;
            }
            return true;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  protected AbstractDrivebase(Distance trackWidthMeters) {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(), 0, 0, new Pose2d());

    // TODO: Move drive base dimensions into new data from the subclasses
    m_driveBaseLengthWithBumpers = Inches.of(29);
    m_driveBaseWidthWithBumpers = Inches.of(26);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
        getLeftEncoder_HAL().getVelocity(), getRightEncoder_HAL().getVelocity()));
    // System.out.println(speeds);
    return speeds;
  }

  public final void stop() {
    setSpeeds(0, 0);
  }

  public final void arcadeDrive(LinearVelocity xSpeed, AngularVelocity rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ZERO_MPS, rot)));
  }

  public final void setSpeeds(ChassisSpeeds chassisSpeeds) {
    setSpeeds(m_kinematics.toWheelSpeeds(chassisSpeeds));
  }

  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds_HAL(speeds);
  }

  public void setSpeeds(double percentage) {
    setSpeeds(percentage, percentage);
  }

  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    // feeder command into the HAL (hardware access layer) for left and right
    // voltages
    if (ENABLE_VOLTAGE_APPLICATON) { // what the hell is ENABLE_VOLTAGE_APPLICATION??
      this.setSpeeds_HAL(leftVoltage, rightVoltage);
    }
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    this.setSpeeds_HAL(leftSpeed, rightSpeed);
  }

  public Distance getLengthIncludingBumpers() {
    return m_driveBaseLengthWithBumpers;
  }

  public Distance getWidthIncludingBumpers() {
    return m_driveBaseWidthWithBumpers;
  }

  public Angle getHeading() {
    return Degrees.of(getOdometry().getPoseMeters().getRotation().getDegrees());
  }

  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void updateOdometry() {
    final Rotation2d rotation = getGyro_HAL().getRotation2d();
    final Distance leftDistanceMeters = getLeftEncoder_HAL().getPosition();
    final Distance rightDistanceMeters = getRightEncoder_HAL().getPosition();
    m_odometry.update(rotation, leftDistanceMeters.in(Meters), rightDistanceMeters.in(Meters));
    m_poseEstimator.update(rotation, leftDistanceMeters.in(Meters), rightDistanceMeters.in(Meters));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
    getGyro_HAL().reset();
    m_odometry.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
    m_poseEstimator.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
  }

  @Override
  public void periodic() {
    Pose2d pose = getPose();

    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());

    SmartDashboard.putNumber("Angle", pose.getRotation().getDegrees());

    SmartDashboard.putNumber(
        "Left velocity", getLeftEncoder_HAL().getVelocity().in(MetersPerSecond));
    SmartDashboard.putNumber(
        "Right velocity", getRightEncoder_HAL().getVelocity().in(MetersPerSecond));

    getRobotRelativeSpeeds();

    updateOdometry();
    // This method will be called once per scheduler run
  }

  protected abstract TrivialEncoder getLeftEncoder_HAL();

  protected abstract TrivialEncoder getRightEncoder_HAL();

  public abstract double getLeftDistanceMeters();

  public abstract double getRightDistanceMeters();

  protected abstract IGyro getGyro_HAL();

  protected abstract void setMotorVoltages_HAL(double leftSpeeds, double rightSpeeds);

  protected abstract void setSpeeds_HAL(double leftSpeeds, double rightSpeeds);

  protected abstract void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds);

  public Distance getLeftDistance() {
    return Meters.of(getLeftDistanceMeters());
  }

  public Distance getRightDistance() {
    return Meters.of(getRightDistanceMeters());
  }
}
