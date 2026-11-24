package first.robot.mechanisms;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N7;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.simulation.EncoderSim;
import org.wpilib.simulation.OnboardIMUSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;

public class SimulatedDriveMechanism extends DriveMechanism {
  // Drivetrain simulation physical parameters
  private static final int MOTORS_PER_SIDE = 2;
  private static final MomentOfInertia ROBOT_MOMENT_OF_INERTIA =
      KilogramSquareMeters.of(4.5);
  private static final Mass ROBOT_MASS = Kilograms.of(45.0);
  private static final Time SIM_LOOP_PERIOD = Milliseconds.of(20.0);

  // Simulated 3% right-wheel slip (97% effective traction) so students can
  // observe odometry drift
  private static final double RIGHT_WHEEL_SLIP_SCALE = 0.97;

  // Measurement standard deviations: [x (m), y (m), heading (rad), vl (m/s), vr
  // (m/s), posl (m), posr (m)]
  private static final Matrix<N7, N1> ENABLED_MEASUREMENT_STD_DEVS =
      VecBuilder.fill(0.002, 0.002, 0.001, 0.05, 0.05, 0.005, 0.005);
  private static final Matrix<N7, N1> ZERO_MEASUREMENT_STD_DEVS =
      VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Static toggle evaluated when SimulatedDriveMechanism is instantiated
  private static boolean enableNoise = false;

  public static boolean isNoiseEnabled() {
    return enableNoise;
  }

  public static void setNoiseEnabled(boolean enabled) {
    enableNoise = enabled;
  }

  private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  private final DifferentialDrivetrainSim driveSim;

  public SimulatedDriveMechanism(
      DoubleSupplier leftSpeedSupplier, DoubleSupplier rightSpeedSupplier) {
    super(leftSpeedSupplier, rightSpeedSupplier);

    driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(MOTORS_PER_SIDE),
        GEAR_RATIO, ROBOT_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
        ROBOT_MASS.in(Kilograms), WHEEL_RADIUS.in(Meters),
        TRACK_WIDTH.in(Meters),
        enableNoise ? ENABLED_MEASUREMENT_STD_DEVS : ZERO_MEASUREMENT_STD_DEVS);

    Scheduler.getDefault().addPeriodic(this::updateSimulation);
  }

  private void updateSimulation() {
    double batteryVoltage = RobotController.getBatteryVoltage();
    driveSim.setInputs(leftLeader.getThrottle() * batteryVoltage,
        rightLeader.getThrottle() * batteryVoltage);

    driveSim.update(SIM_LOOP_PERIOD.in(Seconds));

    // Feed simulated sensor state (applying RIGHT_WHEEL_SLIP_SCALE to
    // demonstrate odometry drift)
    leftEncoderSim.setDistance(driveSim.getLeftPosition());
    leftEncoderSim.setRate(driveSim.getLeftVelocity());
    rightEncoderSim.setDistance(
        driveSim.getRightPosition() * RIGHT_WHEEL_SLIP_SCALE);
    rightEncoderSim.setRate(
        driveSim.getRightVelocity() * RIGHT_WHEEL_SLIP_SCALE);
    OnboardIMUSim.setYaw(driveSim.getHeading().getRadians());

    field.getObject("GroundTruth").setPose(driveSim.getPose());

    // TODO(Simulated Vision Pose Fusion): Once vision-based estimation is
    // implemented:
    // 1. Obtain the current simulation timestamp via
    // org.wpilib.system.Timer.getTimestamp()
    //    (note: RobotController.getFPGATime() was removed in WPILib 2027).
    // 2. Periodically (e.g., at 5–20 Hz, or via PhotonVision's VisionSystemSim
    // fed with
    //    driveSim.getPose()) generate a simulated camera pose estimate from
    //    driveSim.getPose() with realistic Gaussian translation/rotation noise
    //    and simulated pipeline latency.
    // 3. Plot the raw vision estimate on field.getObject("VisionEstimate") and
    // call
    //    addVisionMeasurement(simulatedVisionPose,
    //    Seconds.of(captureTimestampSeconds)) to verify that
    //    DifferentialDrivePoseEstimator corrects the wheel-slip drift.
  }

  @Override
  public void resetPose(Pose2d pose) {
    driveSim.setPose(pose);
    field.getObject("GroundTruth").setPose(pose);
    super.resetPose(pose);
  }

  // =========================================================================
  // WPILib 2027 Telemetry Override (Ground Truth & Drift Metrics)
  // =========================================================================

  @Override
  public void logTo(TelemetryTable table) {
    super.logTo(table);

    table.log("Simulation/Noise Enabled", enableNoise);
    table.log("Simulation/True Pose", driveSim.getPose(), Pose2d.struct);
    table.log(
        "Simulation/True Heading (deg)", driveSim.getHeading().getDegrees());
    table.log("Simulation/Position Drift (m)",
        getPose().getTranslation().getDistance(
            driveSim.getPose().getTranslation()));
    table.log("Simulation/Heading Drift (deg)",
        getPose()
            .getRotation()
            .minus(driveSim.getPose().getRotation())
            .getDegrees());
  }
}
