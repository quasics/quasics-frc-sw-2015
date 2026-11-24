package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drivers.motor.PWMSparkMax;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.DifferentialDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * Tank drivetrain mechanism supporting teleop driving, closed-loop
 * distance/pose commands, pose estimation, and WPILib 2027 telemetry logging.
 *
 * <h2>Future Enhancements</h2>
 * <ul>
 *   <li><b>Optional Closed-Loop Wheel Velocity Control (Feedforward + Velocity
 * PID):</b> Upgrade autonomous distance and pose-navigation commands from
 * open-loop duty-cycle throttle percentages to physical velocity targets
 * ({@code LinearVelocity} and
 *       {@code AngularVelocity}):
 *       <ol>
 *         <li>Characterize the drivetrain with SysId to obtain static friction
 * ({@code kS}), velocity ({@code kV}), and acceleration ({@code kA})
 * constants.</li> <li>Instantiate {@code DifferentialDriveKinematics} (using
 * {@code TRACK_WIDTH}),
 *             {@code SimpleMotorFeedforward}, and left/right wheel velocity
 * {@code PIDController} instances.</li> <li>Add {@code
 * driveWheelSpeeds(LinearVelocity left, LinearVelocity right)} and
 *             {@code driveChassisSpeeds(ChassisSpeeds)} helpers that convert
 * target wheel speeds into feedforward + PID voltage commands normalized by
 *             {@code RobotController.getBatteryVoltage()}.</li>
 *         <li>Update {@code driveDistanceCommand} and {@code
 * driveToPoseCommand} to output target linear and angular velocities instead of
 * duty-cycle throttle values, and log target vs. actual wheel speeds in {@code
 * logTo(...)}.</li>
 *       </ol>
 *   </li>
 *   <li><b>Vision-Based Pose Fusion:</b> Connect an AprilTag coprocessor
 * (PhotonVision or Limelight) and call {@link #addVisionMeasurement(Pose2d,
 * Time)} with latency-compensated timestamps from {@code Timer.getTimestamp()}
 * to correct cumulative wheel-slip drift.</li>
 * </ul>
 */
public class DriveMechanism implements Mechanism, TelemetryLoggable {
  // Physical drivetrain geometry
  protected static final Distance TRACK_WIDTH = Inches.of(21.5);
  protected static final Distance WHEEL_RADIUS = Inches.of(3.0);
  protected static final double GEAR_RATIO = 8.45;
  private static final int ENCODER_CPR = 360;

  // PWM motor channels & DIO encoder channels
  private static final int LEFT_LEADER_PWM_PORT = 0;
  private static final int LEFT_FOLLOWER_PWM_PORT = 1;
  private static final int RIGHT_LEADER_PWM_PORT = 2;
  private static final int RIGHT_FOLLOWER_PWM_PORT = 3;

  private static final int LEFT_ENCODER_CHANNEL_A = 0;
  private static final int LEFT_ENCODER_CHANNEL_B = 1;
  private static final int RIGHT_ENCODER_CHANNEL_A = 2;
  private static final int RIGHT_ENCODER_CHANNEL_B = 3;

  // Pose estimator trust standard deviations: [x (m), y (m), theta (rad)]
  private static final Matrix<N3, N1> STATE_STD_DEVS =
      VecBuilder.fill(Meters.of(0.05).in(Meters), Meters.of(0.05).in(Meters),
          Degrees.of(2.0).in(Radians));
  private static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
      VecBuilder.fill(Meters.of(0.30).in(Meters), Meters.of(0.30).in(Meters),
          Degrees.of(15.0).in(Radians));

  // Drive-to-pose closed-loop control constants
  private static final Distance POSITION_TOLERANCE = Meters.of(0.05);
  private static final Angle HEADING_TOLERANCE = Degrees.of(2.0);
  private static final int HEADING_SETTLE_ITERATIONS = 2;
  private static final double MAX_DRIVE_OUTPUT = 0.65;
  private static final double MAX_TURN_OUTPUT = 0.50;
  private static final double TURN_KS = 0.06;
  private static final Angle REVERSE_HEADING_THRESHOLD = Degrees.of(90.0);

  private static final double DISTANCE_KP = 1.8;
  private static final double DISTANCE_KI = 0.0;
  private static final double DISTANCE_KD = 0.1;

  private static final double HEADING_KP = 0.032;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.005;
  private static final Angle MIN_HEADING_INPUT = Degrees.of(-180.0);
  private static final Angle MAX_HEADING_INPUT = Degrees.of(180.0);

  // Command safety timeouts
  private static final Time DRIVE_DISTANCE_TIMEOUT = Seconds.of(5.0);
  private static final Time DRIVE_TO_POSE_TIMEOUT = Seconds.of(10.0);

  // Protected hardware so SimulatedDriveMechanism can read throttle outputs and
  // inject sensor state
  protected final PWMSparkMax leftLeader =
      new PWMSparkMax(LEFT_LEADER_PWM_PORT);
  private final PWMSparkMax leftFollower =
      new PWMSparkMax(LEFT_FOLLOWER_PWM_PORT);
  protected final PWMSparkMax rightLeader =
      new PWMSparkMax(RIGHT_LEADER_PWM_PORT);
  private final PWMSparkMax rightFollower =
      new PWMSparkMax(RIGHT_FOLLOWER_PWM_PORT);
  private final DifferentialDrive drive =
      new DifferentialDrive(leftLeader::setThrottle, rightLeader::setThrottle);

  protected final Encoder leftEncoder =
      new Encoder(LEFT_ENCODER_CHANNEL_A, LEFT_ENCODER_CHANNEL_B);
  protected final Encoder rightEncoder =
      new Encoder(RIGHT_ENCODER_CHANNEL_A, RIGHT_ENCODER_CHANNEL_B);

  // TODO(Hardware Confirmation): Verify physical SystemCore mounting
  // orientation and axis conventions.
  // 1. Inspect the physical SystemCore controller on the chassis: confirm it is
  // mounted flat
  //    (parallel to the floor) rather than vertically on a wall/bellypan. If
  //    mounted vertically or upside down, change MountOrientation.FLAT to match
  //    the hardware orientation.
  // 2. Rotate the robot counter-clockwise (CCW) by 90 degrees on the floor and
  // check
  //    "Drive/Odometry/Heading (deg)" in telemetry to confirm yaw increases
  //    positively (+90 deg) per WPILib NWU conventions.
  protected final OnboardIMU imu =
      new OnboardIMU(OnboardIMU.MountOrientation.FLAT);

  private final DifferentialDrivePoseEstimator poseEstimator;
  protected final Field2d field = new Field2d();

  private final DoubleSupplier leftSpeedSupplier;
  private final DoubleSupplier rightSpeedSupplier;

  public DriveMechanism(
      DoubleSupplier leftSpeedSupplier, DoubleSupplier rightSpeedSupplier) {
    this.leftSpeedSupplier = leftSpeedSupplier;
    this.rightSpeedSupplier = rightSpeedSupplier;

    leftLeader.addFollower(leftFollower);
    rightLeader.addFollower(rightFollower);
    rightLeader.setInverted(true);

    Distance distancePerPulse =
        WHEEL_RADIUS.times(2.0 * Math.PI).div(ENCODER_CPR);
    leftEncoder.setDistancePerPulse(distancePerPulse.in(Meters));
    rightEncoder.setDistancePerPulse(distancePerPulse.in(Meters));

    poseEstimator = new DifferentialDrivePoseEstimator(imu.getRotation2d(),
        leftEncoder.getDistance(), rightEncoder.getDistance(), new Pose2d(),
        STATE_STD_DEVS, VISION_MEASUREMENT_STD_DEVS);

    Scheduler.getDefault().addPeriodic(this::updateOdometry);

    // Default command drives using the injected left/right joystick suppliers
    setDefaultCommand(tankDriveCommand());
  }

  private void updateOdometry() {
    poseEstimator.update(imu.getRotation2d(), leftEncoder.getDistance(),
        rightEncoder.getDistance());
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Protected state mutators: active OpModes schedule resetPoseCommand(...) or
  // stopCommand()
  protected void resetPose(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    poseEstimator.resetPosition(imu.getRotation2d(), 0.0, 0.0, pose);
    field.setRobotPose(pose);
  }

  protected void stop() {
    drive.stopMotor();
  }

  /**
   * Synchronizes the robot's starting pose on {@link Field2d} and pose
   * estimator while the robot is disabled or disconnected (prior to an OpMode
   * being enabled). Ignored while enabled so active commands retain exclusive
   * ownership of drivetrain state.
   */
  public void syncInitialPose(Pose2d pose) {
    if (!RobotState.isEnabled()) {
      resetPose(pose);
    }
  }

  // TODO(Vision Pose Estimation): Connect a vision coprocessor (e.g.,
  // PhotonVision or Limelight) to fuse field-relative AprilTag pose estimates
  // into DifferentialDrivePoseEstimator:
  // 1. Measure the physical camera-to-robot Transform3d (X forward, Y left, Z
  // up in meters, plus
  //    roll/pitch/yaw in NWU coordinates) and apply it when transforming camera
  //    poses to robot poses.
  // 2. Capture the exact frame capture timestamp using WPILib 2027's
  // Timer.getTimestamp()
  //    (subtracting pipeline latency) so DifferentialDrivePoseEstimator can
  //    replay odometry history accurately and avoid latency-induced pose lag.
  // 3. Filter out high-ambiguity single-tag detections (e.g., poseAmbiguity >
  // 0.2) and dynamically
  //    scale visionMeasurementStdDevs based on tag count and average distance
  //    to tags.
  // 4. Call addVisionMeasurement(...) from the vision subsystem/periodic loop
  // whenever a valid
  //    multi-tag or low-ambiguity target frame arrives.

  /**
   * Fuses an external vision pose measurement (e.g. AprilTag pipeline) into
   * pose estimation.
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, Time timestamp) {
    poseEstimator.addVisionMeasurement(visionRobotPose, timestamp.in(Seconds));
  }

  /**
   * Fuses an external vision pose measurement with custom trust standard
   * deviations.
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, Time timestamp,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPose, timestamp.in(Seconds), visionMeasurementStdDevs);
  }

  // =========================================================================
  // WPILib 2027 TelemetryLoggable Implementation
  // =========================================================================

  @Override
  public void logTo(TelemetryTable table) {
    table.log("Odometry/Pose", getPose(), Pose2d.struct);
    table.log("Odometry/X (m)", getPose().getX());
    table.log("Odometry/Y (m)", getPose().getY());
    table.log("Odometry/Heading (deg)", getPose().getRotation().getDegrees());

    table.log("Encoders/Left Distance (m)", leftEncoder.getDistance());
    table.log("Encoders/Right Distance (m)", rightEncoder.getDistance());

    Telemetry.log("Drive/Field", field);
  }

  // =========================================================================
  // Commands v3 Factory Methods
  // =========================================================================

  /** Stops all drivetrain motors through the mechanism scheduler. */
  public Command stopCommand() {
    return run(coroutine -> stop()).named("Drive.Stop");
  }

  /**
   * Resets encoder distances and odometry pose through the mechanism
   * scheduler.
   */
  public Command resetPoseCommand(Pose2d pose) {
    return run(coroutine -> resetPose(pose)).named("Drive.ResetPose");
  }

  /**
   * Continuous tank drive command using the injected left and right speed
   * suppliers.
   */
  public Command tankDriveCommand() {
    return tankDriveCommand(leftSpeedSupplier, rightSpeedSupplier);
  }

  /**
   * Continuous tank drive command using explicit left and right speed
   * suppliers.
   */
  public Command tankDriveCommand(
      DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    return runRepeatedly(()
                             -> drive.tankDrive(leftSpeed.getAsDouble(),
                                 rightSpeed.getAsDouble()))
        .named("Drive.TankDrive");
  }

  /**
   * Drives straight for a given distance (supports positive forward or
   * negative reverse).
   */
  public Command driveDistanceCommand(Distance distance, double speed) {
    return run(coroutine -> {
      double targetMeters = distance.in(Meters);
      double startDist = leftEncoder.getDistance();
      double directedSpeed = Math.copySign(speed, targetMeters);
      while (Math.abs(leftEncoder.getDistance() - startDist)
          < Math.abs(targetMeters)) {
        drive.tankDrive(directedSpeed, directedSpeed);
        coroutine.yield();
      }
      stop();
    })
        .named("Drive.DriveDistance")
        .withTimeout(DRIVE_DISTANCE_TIMEOUT);
  }

  /**
   * Commands v3 coroutine that drives the tank drivetrain to a field-relative
   * target Pose2d and aligns to its target rotation.
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    return driveToPoseCommand(targetPose, true);
  }

  /**
   * Commands v3 coroutine that drives the tank drivetrain to a field-relative
   * target Pose2d using pose estimation. Latches the forward/reverse direction
   * once when Stage 1 begins so the drivetrain never chatters near 90 degrees.
   *
   * @param targetPose Target field-relative pose.
   * @param alignFinalHeading If true, rotates in place at the destination to
   *     match
   *     {@code targetPose.getRotation()}; if false, finishes as soon as the (X,
   * Y) position tolerance is reached (useful for pass-through intermediate
   * waypoints).
   */
  public Command driveToPoseCommand(
      Pose2d targetPose, boolean alignFinalHeading) {
    return run(coroutine -> {
      PIDController distancePid =
          new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
      PIDController headingPid =
          new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
      headingPid.enableContinuousInput(
          MIN_HEADING_INPUT.in(Degrees), MAX_HEADING_INPUT.in(Degrees));

      field.getObject("TargetPose").setPose(targetPose);

      // Latch travel direction once when Stage 1 begins
      Pose2d initialPose = getPose();
      Translation2d initialToTarget =
          targetPose.getTranslation().minus(initialPose.getTranslation());
      double initialHeadingErrorDeg = initialToTarget.getAngle()
                                          .orElse(initialPose.getRotation())
                                          .minus(initialPose.getRotation())
                                          .getDegrees();

      final boolean driveInReverse = Math.abs(initialHeadingErrorDeg)
          > REVERSE_HEADING_THRESHOLD.in(Degrees);
      final double directionSign = driveInReverse ? -1.0 : 1.0;

      // Stage 1: Drive to the target (X, Y) translation (latched direction)
      while (true) {
        Pose2d currentPose = getPose();
        Translation2d toTarget =
            targetPose.getTranslation().minus(currentPose.getTranslation());
        double distanceError = toTarget.getNorm();

        if (distanceError <= POSITION_TOLERANCE.in(Meters)) {
          break;
        }

        Rotation2d bearingToTarget =
            toTarget.getAngle().orElse(currentPose.getRotation());
        Rotation2d effectiveRobotHeading = driveInReverse
            ? currentPose.getRotation().plus(Rotation2d.k180deg)
            : currentPose.getRotation();
        double effectiveHeadingErrorDeg =
            bearingToTarget.minus(effectiveRobotHeading).getDegrees();

        double alignmentScale =
            Math.max(0.0, Math.cos(Math.toRadians(effectiveHeadingErrorDeg)));
        double forwardSpeed = directionSign
            * Math.clamp(
                distancePid.calculate(0.0, distanceError) * alignmentScale, 0.0,
                MAX_DRIVE_OUTPUT);

        double turnSpeed =
            Math.clamp(headingPid.calculate(0.0, effectiveHeadingErrorDeg),
                -MAX_TURN_OUTPUT, MAX_TURN_OUTPUT);

        drive.arcadeDrive(forwardSpeed, turnSpeed);
        coroutine.yield();
      }

      // Stage 2: Optionally rotate in place to match targetPose.getRotation()
      if (alignFinalHeading) {
        headingPid.reset();
        int settledCount = 0;
        while (true) {
          Pose2d currentPose = getPose();
          double finalHeadingErrorDeg = targetPose.getRotation()
                                            .minus(currentPose.getRotation())
                                            .getDegrees();

          if (Math.abs(finalHeadingErrorDeg) <= HEADING_TOLERANCE.in(Degrees)) {
            settledCount++;
            if (settledCount >= HEADING_SETTLE_ITERATIONS) {
              break;
            }
            stop();
            coroutine.yield();
            continue;
          }

          settledCount = 0;
          double pidOutput = headingPid.calculate(0.0, finalHeadingErrorDeg);
          double feedforward = Math.copySign(TURN_KS, finalHeadingErrorDeg);
          double turnSpeed = Math.clamp(
              pidOutput + feedforward, -MAX_TURN_OUTPUT, MAX_TURN_OUTPUT);

          drive.arcadeDrive(0.0, turnSpeed);
          coroutine.yield();
        }
      }

      stop();
    })
        .named("Drive.DriveToPose")
        .withTimeout(DRIVE_TO_POSE_TIMEOUT);
  }
}
