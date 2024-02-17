// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.DeadbandEnforcer;
import frc.robot.utils.RobotSettings;

/**
 * Defines an abstract base class for common "drivebase functionality", allowing
 * hardware-specific functionality to be isolated behind a Hardware Access Layer
 * (HAL). This makes it easier to switch between running on a "big bot", vs.
 * running on a "little bot" like an XRP, vs. running purely within the
 * simulator.
 */
public abstract class AbstractDrivebase extends SubsystemBase {
  /**
   * The key that will be used in posting the estimated pose (as a Pose2d object)
   * to the BulletinBoard.
   */
  public static final String BULLETIN_BOARD_POSE_KEY = "Drivebase.Pose";

  /** Maximum linear speed is 3 meters per second. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(3.0);

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  final static DeadbandEnforcer wheelSpeedsDeadband = new DeadbandEnforcer(0.1);

  final static DeadbandEnforcer drivePercentageDeadband = new DeadbandEnforcer(0.05);

  private static final boolean ENABLE_VOLTAGE_APPLICATON = true;

  protected static final boolean LOG_TO_SMARTDASHBOARD = false;

  protected static final Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);

  // TODO: Consider adjusting this max output to prevent running @ 100%.
  private static double MOTORS_PERCENT_MAX_OUTPUT = 1.0;

  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final Measure<Distance> m_driveBaseLengthWithBumpers;
  private final Measure<Distance> m_driveBaseWidthWithBumpers;

  /**
   * Constructor.
   * 
   * @param robot specifies what robot we're trying to build (and its settings)
   */
  public AbstractDrivebase(RobotSettings.Robot robot) {
    this(robot.trackWidthMeters, robot.kP, robot.kI, robot.kD, robot.kS, robot.kV, robot.kA);
    super.setName(robot.name());
  }

  /**
   * Constructor.
   *
   * @param trackWidthMeters track width (from SysID using the "Drivetrain
   *                         (Angular)" test)
   * @param kP               kP value for PID control of motors
   * @param kI               kI value for PID control of motors
   * @param kD               kD value for PID control of motors
   * @param kS               voltage needed to overcome the drive motors' static
   *                         friction
   * @param kV               voltage scaling value used to hold a given velocity
   * @param kA               voltage scaling value used to hold a given
   *                         acceleration
   *
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#track-width
   */
  protected AbstractDrivebase(
      Measure<Distance> trackWidthMeters,
      double kP, double kI, double kD,
      Measure<Voltage> kS, Measure<Per<Voltage, Velocity<Distance>>> kV,
      Measure<Per<Voltage, Velocity<Velocity<Distance>>>> kA) {
    m_leftPIDController = new PIDController(kP, kI, kD);
    m_rightPIDController = new PIDController(kP, kI, kD);
    m_feedforward = new SimpleMotorFeedforward(kS.in(Volts), kV.in(VoltsPerMeterPerSecond),
        kA.in(VoltsPerMeterPerSecondSquared));
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, new Rotation2d(), /* leftDistanceMeters */ 0,
        /* rightDistanceMeters */ 0, /* initialPostMeters */ new Pose2d());

    // TODO: Move drive base dimensions into new data from the subclasses
    m_driveBaseLengthWithBumpers = Inches.of(29);
    m_driveBaseWidthWithBumpers = Inches.of(26);
  }

  public Measure<Distance> getLengthIncludingBumpers() {
    return m_driveBaseLengthWithBumpers;
  }

  public Measure<Distance> getWidthIncludingBumpers() {
    return m_driveBaseWidthWithBumpers;
  }

  public Measure<Distance> getLeftDistance() {
    return getLeftEncoder_HAL().getPosition();
  }

  public Measure<Distance> getRightDistance() {
    return getRightEncoder_HAL().getPosition();
  }

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SimpleMotorFeedforward getMotorFeedforward() {
    return m_feedforward;
  }

  public double getKP() {
    return m_leftPIDController.getP();
  }

  public double getKI() {
    return m_leftPIDController.getI();
  }

  public double getKD() {
    return m_leftPIDController.getD();
  }

  /** @return current wheel speeds (in m/s) */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoder_HAL().getVelocity(),
        getRightEncoder_HAL().getVelocity());
  }

  /** Update the robot's odometry. */
  public void updateOdometry() {
    final Rotation2d rotation = getGyro_HAL().getRotation2d();
    final Measure<Distance> leftDistanceMeters = getLeftEncoder_HAL().getPosition();
    final Measure<Distance> rightDistanceMeters = getRightEncoder_HAL().getPosition();
    getOdometry().update(rotation, leftDistanceMeters.in(Meters),
        rightDistanceMeters.in(Meters));
    m_poseEstimator.update(rotation, leftDistanceMeters.in(Meters),
        rightDistanceMeters.in(Meters));
  }

  /** Get the current robot pose, based on odometery. */
  public Pose2d getPose() {
    return getOdometry().getPoseMeters();
  }

  /**
   * Get the current estimated robot pose, based on odometery, plus any vision
   * updates.
   */
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets robot odometry (e.g., if we know that we've been placed at a
   * specific position/angle on the field, such as at the start of a match).
   */
  public void resetOdometry(Pose2d pose) {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
    getOdometry().resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
    m_poseEstimator.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
  }

  public void integrateVisionMeasurement(Pose2d pose, double timestampSeconds) {
    /**
     * TODO: Update code to make it more robust w.r.t. bad vision data.
     *
     * From the docs: "To promote stability of the pose estimate and make it
     * robust to bad vision data, we recommend only adding vision measurements
     * that are already within one meter or so of the current pose estimate."
     *
     * Another option is to downgrade the assumed reliability of the
     * measurement, based on the distance. (See
     * https://www.chiefdelphi.com/t/photonvision-swerveposeestimator-produces-wrong-pose/430906/6)
     * 
     * Some possible considerations from the CD thread:
     * * It is much more reliable if you only calculate poses when you see more than
     * one tag (hence SolvePnP)
     * * Vision poses are much more accurate if the cameras are not perpendicular to
     * the tags (the more angle the better, to a certain extent)
     * * Using the distance from the tag as standard deviations helps to trust data
     * closer to the tag - we use poseEstimator.addVisionMeasurement(pose,
     * timestamp, VecBuilder.fill(distance / 2, distance / 2, 100));
     */
    // Figure out how far we *think* we are
    Pose2d currentEstimate = m_poseEstimator.getEstimatedPosition();
    var transform = currentEstimate.minus(pose);
    final double distance = Math.sqrt(transform.getX() * transform.getX() +
        transform.getY() * transform.getY());

    // Add in the vision-based estimate, using the distance as a "trust-scaling"
    // factor.
    m_poseEstimator.addVisionMeasurement(
        pose, timestampSeconds,
        VecBuilder.fill(distance / 2, distance / 2, 100));
  }

  public final void stop() {
    setSpeedsImpl(0, 0, false);
  }

  void logValue(String label, double val) {
    if (LOG_TO_SMARTDASHBOARD) {
      SmartDashboard.putNumber(label, val);
    }
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis (in m/s)
   * @param rot    the rotation (in radians/s)
   */
  public final void arcadeDrive(Measure<Velocity<Distance>> xSpeed,
      Measure<Velocity<Angle>> rot) {
    logValue("xSpeed", xSpeed.in(MetersPerSecond));
    logValue("rotSpeed", rot.in(RadiansPerSecond));

    // Cap requested speeds to the maxima.
    if (xSpeed.gt(MAX_SPEED)) {
      xSpeed = MAX_SPEED;
    } else if (xSpeed.lt(MAX_SPEED.negate())) {
      xSpeed = MAX_SPEED.negate();
    }
    if (rot.gt(MAX_ANGULAR_SPEED)) {
      rot = MAX_ANGULAR_SPEED;
    } else if (rot.lt(MAX_ANGULAR_SPEED.negate())) {
      rot = MAX_ANGULAR_SPEED.negate();
    }
    logValue("xSpeed (capped)", xSpeed.in(MetersPerSecond));
    logValue("rotSpeed (capped)", rot.in(RadiansPerSecond));

    setSpeeds(
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ZERO_MPS, rot)));
  }

  /** Sets speeds for the drivetrain motors. */
  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeedsImpl(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond,
        true);
  }

  /** Sets speeds for the drivetrain motors. */
  public final void setSpeedsImpl(double leftMetersPerSecond,
      double rightMetersPerSecond,
      boolean includePid) {
    logValue("leftSpeed", leftMetersPerSecond);
    logValue("rightSpeed", rightMetersPerSecond);

    var leftStabilized = wheelSpeedsDeadband.limit(leftMetersPerSecond);
    var rightStabilized = wheelSpeedsDeadband.limit(rightMetersPerSecond);
    logValue("leftStable", leftStabilized);
    logValue("rightStable", rightStabilized);

    // Figure out the voltages we should need at the target speeds.
    var leftFeedforward = m_feedforward.calculate(leftStabilized);
    var rightFeedforward = m_feedforward.calculate(rightStabilized);
    logValue("leftFF", leftFeedforward);
    logValue("rightFF", rightFeedforward);

    // Figure out the deltas, based on our current speed vs. the target speeds.
    double leftPidOutput = includePid ? m_leftPIDController.calculate(
        getLeftEncoder_HAL().getVelocity().in(MetersPerSecond),
        leftStabilized)
        : 0;
    double rightPidOutput = includePid ? m_rightPIDController.calculate(
        getRightEncoder_HAL().getVelocity().in(MetersPerSecond),
        rightStabilized)
        : 0;
    logValue("leftPid", leftPidOutput);
    logValue("rightPid", rightPidOutput);

    // OK, apply those to the actual hardware.
    setMotorVoltages(leftFeedforward + leftPidOutput,
        rightFeedforward + rightPidOutput);
  }

  /**
   * Applies the specified voltages to the motors (and remembers what was set,
   * so that we can periodically update it, as required for voltage compensation
   * to work properly).
   *
   * @see edu.wpi.first.wpilibj.motorcontrol.MotorController#setVoltage
   */
  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    if (ENABLE_VOLTAGE_APPLICATON) {
      this.setMotorVoltages_HAL(leftVoltage, rightVoltage);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();

    // If an estimated position has been posted by the vision subsystem, integrate
    // it into our estimate.
    Optional<Object> optionalPose = BulletinBoard.getValue(VisionSubsystem.BULLETIN_BOARD_POSE_KEY, Pose2d.class);
    optionalPose.ifPresent(poseObject -> {
      BulletinBoard.getValue(VisionSubsystem.BULLETIN_BOARD_TIMESTAMP_KEY,
          Double.class)
          .ifPresent(timestampObject -> integrateVisionMeasurement((Pose2d) poseObject, (Double) timestampObject));
    });

    // Publish our estimated position
    BulletinBoard.updateValue(BULLETIN_BOARD_POSE_KEY, getEstimatedPose());
  }

  /** Prevents us from pushing voltage/speed values too small for the motors. */
  final static DeadbandEnforcer m_voltageDeadbandEnforcer = new DeadbandEnforcer(-0.001);

  public static double convertVoltageToPercentSpeed(double volts) {
    final double inputVoltage = RobotController.getInputVoltage();
    final double percentMaxVoltage = (volts / inputVoltage);
    final double speedPercentage = m_voltageDeadbandEnforcer.limit(
        // Use the % of max voltage
        percentMaxVoltage
    // Note: originally used - m_voltageDeadbandEnforcer.limit(mps /
    // MAX_SPEED);
    );
    return speedPercentage;
  }

  // TODO: Think about replacing "double" with something type-safe. (Using
  // Measure<Dimensionless> won't work unless I change the function name.)
  public void arcadeDrive(double xSpeed, double rotationSpeed, boolean squareInputs) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(xSpeed, rotationSpeed, squareInputs);

    double adjustedLeftPercent = drivePercentageDeadband.limit(speeds.left * MOTORS_PERCENT_MAX_OUTPUT);
    double adjustedRightPercent = drivePercentageDeadband.limit(speeds.right * MOTORS_PERCENT_MAX_OUTPUT);
    tankDrivePercent_HAL(adjustedLeftPercent, adjustedRightPercent);
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Hardware abstraction layer definition
  //
  /////////////////////////////////////////////////////////////////////////////////

  protected abstract TrivialEncoder getLeftEncoder_HAL();

  protected abstract TrivialEncoder getRightEncoder_HAL();

  protected abstract IGyro getGyro_HAL();

  protected abstract double getLeftSpeedPercentage_HAL();

  protected abstract double getRightSpeedPercentage_HAL();

  protected abstract void tankDrivePercent_HAL(double leftPercent, double rightPercent);

  /**
   * Directly sets voltages for left/right motors.
   * 
   * @param leftVoltage  voltage for left-side motors
   * @param rightVoltage voltage for right-side motors
   */
  protected abstract void setMotorVoltages_HAL(double leftVoltage,
      double rightVoltage);

  /////////////////////////////////////////////////////////////////////////////////
  //
  // SysId (profiling) support
  //
  /////////////////////////////////////////////////////////////////////////////////

  // Mutable holder for unit-safe voltage values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step
      // voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            final double rawVolts = volts.in(Volts);
            setMotorVoltages_HAL(rawVolts, rawVolts);
          },
          // Tell SysId how to record a frame of data for each motor on the
          // mechanism being characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder,
            // we consider the entire group to be one motor.
            log.motor("drive-left")
                .voltage(m_appliedVoltage.mut_replace(
                    getLeftSpeedPercentage_HAL() *
                        RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(getLeftEncoder_HAL().getPosition())
                .linearVelocity(getLeftEncoder_HAL().getVelocity());
            // Record a frame for the right motors. Since these share an
            // encoder, we consider the entire group to be one motor.
            log.motor("drive-right")
                .voltage(m_appliedVoltage.mut_replace(
                    getRightSpeedPercentage_HAL() *
                        RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(getRightEncoder_HAL().getPosition())
                .linearVelocity(getRightEncoder_HAL().getVelocity());
          },
          // Tell SysId to make generated commands require this subsystem,
          // suffix test state in WPILog with this subsystem's name ("drive")
          this));

  /**
   * @return a Command for use in running quasistatic profiling in the
   *         specified direction.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * @return a Command for use in running dynamic profiling in the
   *         specified direction.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
