// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

/**
 * An abstract class that represents a drivebase.
 * 
 * Some possible enhancements:
 * <ul>
 * <li>
 * Use either the raw odometry or (unified) pose estimation to provide a signal
 * to the drive team about the robot's position on the field (e.g., when it's
 * oriented towards the barge and close enough to make the shot). This could be
 * done by putting something on the dashboard, changing the lights on the robot,
 * etc.
 * </li>
 * </ul>
 */
public abstract class AbstractDrivebase extends SubsystemBase {
  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3);

  // Max rotational speed is 1/2 rotations per second
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(6.5);

  protected static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final Distance m_driveBaseLengthWithBumpers;
  private final Distance m_driveBaseWidthWithBumpers;
  /*
   * Sally mass: 61.2 lbs
   * Sally moi: 5.4701 kg m^2
   * New bot mass:
   * New bot moi:
   */
  // TODO: add some config thing so these values can be easily changed across
  // robots
  /*
   * final protected PIDController m_leftPidController = new PIDController(1.6018,
   * 0.0, 0.0);
   * final protected PIDController m_rightPidController = new
   * PIDController(1.6018, 0.0, 0.0);
   * final protected DifferentialDriveFeedforward m_feedforward = new
   * DifferentialDriveFeedforward(1.9802, 1.9202, 1.5001,
   * 0.29782);
   */

  private final Vision m_vision = new Vision(this::getPose);

  final protected PIDController m_leftPidController = new PIDController(0.1474, 0.0, 0.0);
  final protected PIDController m_rightPidController = new PIDController(0.16513, 0.0, 0.0);
  final protected DifferentialDriveFeedforward m_feedforward = new DifferentialDriveFeedforward(
      0.18581,
      0.05689,
      0.20933,
      0.054005);

  private final LTVUnicycleController m_controller = new LTVUnicycleController(0.02);

  /** Creates a new IDrivebase. */
  public AbstractDrivebase(RobotSettings.Robot robot) {
    this(robot.trackWidthMeters);
    super.setName(robot.name());
  }

  protected AbstractDrivebase(Distance trackWidthMeters) {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    // TODO: Move drive base dimensions into new data from the subclasses
    m_driveBaseLengthWithBumpers = Inches.of(29);
    m_driveBaseWidthWithBumpers = Inches.of(26);
  }

  public void followTrajectory(DifferentialSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Get the velocity feedforward specified by the sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = m_controller.calculate(
        pose,
        sample.getPose(),
        ff.vxMetersPerSecond,
        ff.omegaRadiansPerSecond);

    // System.out.println("Commandded: " + speeds);
    // System.out.println("Actual: " + getRobotRelativeSpeeds());

    // Apply the generated speeds
    driveWithPid(speeds);
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

  public final void driveWithPid(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    var feedforward = m_feedforward.calculate(getLeftEncoder_HAL().getVelocity().in(MetersPerSecond),
        speeds.leftMetersPerSecond,
        getRightEncoder_HAL().getVelocity().in(MetersPerSecond), speeds.rightMetersPerSecond, 0.02);

    double leftPidOutput = m_leftPidController.calculate(getLeftEncoder_HAL().getVelocity().in(MetersPerSecond),
        speeds.leftMetersPerSecond);
    double rightPidOutput = m_rightPidController.calculate(getRightEncoder_HAL().getVelocity().in(MetersPerSecond),
        speeds.rightMetersPerSecond);

    setMotorVoltages(feedforward.left + leftPidOutput, feedforward.right + rightPidOutput);
  }

  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds_HAL(speeds);
  }

  public void setSpeeds(double percentage) {
    setSpeeds(percentage, percentage);
  }

  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    this.setMotorVoltages_HAL(leftVoltage, rightVoltage);
  }

  public void setMotorVoltages(Voltage leftVoltage, Voltage rightVoltage) {
    this.setMotorVoltages(leftVoltage.in(Volts), rightVoltage.in(Volts));
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

    /*
     * Optional<EstimatedRobotPose> result = m_vision.visionEstimator.update();
     * if (result.isPresent()) {
     * EstimatedRobotPose pose = result.get();
     * Pose2d toPrint = pose.estimatedPose.toPose2d();
     * SmartDashboard.putNumber("returned x", toPrint.getX());
     * SmartDashboard.putNumber("returned y", toPrint.getY());
     * SmartDashboard.putNumber("returned angle",
     * toPrint.getRotation().getDegrees());
     * m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
     * pose.timestampSeconds);
     * 
     * }
     */
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getYawDegrees() {
    return getPose().getRotation().getDegrees();
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    /*
     * getLeftEncoder_HAL().reset();
     * getRightEncoder_HAL().reset();
     * getGyro_HAL().reset();
     * m_odometry.resetPosition(new Rotation2d(), 0, 0, pose);
     * m_poseEstimator.resetPosition(new Rotation2d(), 0, 0, pose);
     */
    final Distance leftPosition = getLeftPosition();
    final Distance rightPosition = getRightPosition();
    final Rotation2d position = getGyro_HAL().getRotation2d();

    getOdometry().resetPosition(position, leftPosition, rightPosition, pose);
    m_poseEstimator.resetPosition(
        position, leftPosition.in(Meters), rightPosition.in(Meters), pose);
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

  protected abstract IGyro getGyro_HAL();

  protected abstract void setMotorVoltages_HAL(double leftSpeeds, double rightSpeeds);

  public abstract Voltage getLeftVoltage();

  public abstract Voltage getRightVoltage();

  protected abstract void setSpeeds_HAL(double leftSpeeds, double rightSpeeds);

  protected abstract void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds);

  public Distance getLeftPosition() {
    return getLeftEncoder_HAL().getPosition();
  }

  public Distance getRightPosition() {
    return getRightEncoder_HAL().getPosition();
  }

  public LinearVelocity getLeftVelocity() {
    return getLeftEncoder_HAL().getVelocity();
  }

  public LinearVelocity getRightVelocity() {
    return getRightEncoder_HAL().getVelocity();
  }
}
