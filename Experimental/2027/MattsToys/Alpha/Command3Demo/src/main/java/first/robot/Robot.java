package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import first.robot.mechanisms.DriveMechanism;
import first.robot.mechanisms.SimulatedDriveMechanism;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.driverstation.Joystick;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;

public class Robot extends OpModeRobot {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final double JOYSTICK_DEADBAND = 0.05;

  // Simulation Keyboard 1 axis & button mappings (when dragged onto
  // Joystick[0])
  private static final int SIM_LEFT_THROTTLE_AXIS = 1; // W/S
  private static final int SIM_RIGHT_THROTTLE_AXIS = 3; // I/K or Up/Down
  private static final int SIM_DRIVE_DISTANCE_BUTTON = 1; // Z
  private static final int SIM_RESET_POSE_BUTTON = 2; // X
  private static final int SIM_DRIVE_TO_POSE_BUTTON = 3; // C

  // Dashboard NetworkTables topics for pre-match starting pose configuration
  private static final String START_POSE_TABLE = "Dashboard/Start Pose";
  private static final String START_X_TOPIC = "X (m)";
  private static final String START_Y_TOPIC = "Y (m)";
  private static final String START_HEADING_TOPIC = "Heading (deg)";

  public static final Pose2d DEFAULT_INITIAL_POSE =
      new Pose2d(Meters.of(2.0), Meters.of(4.0), new Rotation2d());

  public final DriveMechanism driveMechanism;
  public final Trigger driveDistanceTrigger;
  public final Trigger resetPoseTrigger;
  public final Trigger driveToPoseTrigger;

  private final DoubleEntry startXEntry;
  private final DoubleEntry startYEntry;
  private final DoubleEntry startHeadingEntry;

  private Pose2d lastAppliedStartPose = DEFAULT_INITIAL_POSE;
  private boolean wasDisabled = true;

  public Robot() {
    final DoubleSupplier leftSpeedSupplier;
    final DoubleSupplier rightSpeedSupplier;

    if (RobotBase.isReal()) {
      CommandXboxController xbox =
          new CommandXboxController(DRIVER_CONTROLLER_PORT);
      leftSpeedSupplier =
          () -> MathUtil.applyDeadband(-xbox.getLeftY(), JOYSTICK_DEADBAND);
      rightSpeedSupplier =
          () -> MathUtil.applyDeadband(-xbox.getRightY(), JOYSTICK_DEADBAND);

      driveDistanceTrigger = xbox.a();
      resetPoseTrigger = xbox.b();
      driveToPoseTrigger = xbox.x();
    } else {
      Joystick keyboard = new Joystick(DRIVER_CONTROLLER_PORT);
      leftSpeedSupplier = ()
          -> MathUtil.applyDeadband(
              -keyboard.getRawAxis(SIM_LEFT_THROTTLE_AXIS), JOYSTICK_DEADBAND);
      rightSpeedSupplier = ()
          -> MathUtil.applyDeadband(
              -keyboard.getRawAxis(SIM_RIGHT_THROTTLE_AXIS), JOYSTICK_DEADBAND);

      driveDistanceTrigger =
          new Trigger(() -> keyboard.getRawButton(SIM_DRIVE_DISTANCE_BUTTON));
      resetPoseTrigger =
          new Trigger(() -> keyboard.getRawButton(SIM_RESET_POSE_BUTTON));
      driveToPoseTrigger =
          new Trigger(() -> keyboard.getRawButton(SIM_DRIVE_TO_POSE_BUTTON));
    }

    // Publish editable starting pose entries for the drive team
    NetworkTable startPoseTable =
        NetworkTableInstance.getDefault().getTable(START_POSE_TABLE);
    startXEntry = startPoseTable.getDoubleTopic(START_X_TOPIC)
                      .getEntry(DEFAULT_INITIAL_POSE.getX());
    startYEntry = startPoseTable.getDoubleTopic(START_Y_TOPIC)
                      .getEntry(DEFAULT_INITIAL_POSE.getY());
    startHeadingEntry =
        startPoseTable.getDoubleTopic(START_HEADING_TOPIC)
            .getEntry(DEFAULT_INITIAL_POSE.getRotation().getDegrees());

    startXEntry.set(DEFAULT_INITIAL_POSE.getX());
    startYEntry.set(DEFAULT_INITIAL_POSE.getY());
    startHeadingEntry.set(DEFAULT_INITIAL_POSE.getRotation().getDegrees());

    // Optional: enable simulated sensor noise before instantiating
    // SimulatedDriveMechanism SimulatedDriveMechanism.setNoiseEnabled(true);

    driveMechanism = RobotBase.isReal()
        ? new DriveMechanism(leftSpeedSupplier, rightSpeedSupplier)
        : new SimulatedDriveMechanism(leftSpeedSupplier, rightSpeedSupplier);

    // Sync initial starting pose immediately on Field2d before any OpMode is
    // enabled
    driveMechanism.syncInitialPose(DEFAULT_INITIAL_POSE);
  }

  /**
   * Reads the drive team's currently configured starting pose from
   * NetworkTables.
   */
  public Pose2d getConfiguredStartPose() {
    return new Pose2d(Meters.of(startXEntry.get()),
        Meters.of(startYEntry.get()),
        new Rotation2d(Degrees.of(startHeadingEntry.get())));
  }

  @Override
  public void robotPeriodic() {
    // Cancel active commands and stop drivetrain motors immediately upon
    // entering Disabled mode
    boolean isDisabled = RobotState.isDisabled();
    if (isDisabled && !wasDisabled) {
      Scheduler.getDefault().cancelAll();
      Scheduler.getDefault().schedule(driveMechanism.stopCommand());
    }
    wasDisabled = isDisabled;

    // Reflect drive team changes to the starting pose immediately on Field2d
    // while not enabled (covers both Disconnected and Disabled states)
    if (!RobotState.isEnabled()) {
      Pose2d configuredPose = getConfiguredStartPose();
      if (!configuredPose.equals(lastAppliedStartPose)) {
        lastAppliedStartPose = configuredPose;
        driveMechanism.syncInitialPose(configuredPose);
      }
    }

    // Runs active OpMode coroutines, triggers, odometry updates, and simulation
    // physics
    Scheduler.getDefault().run();

    // Flush mechanism telemetry to NetworkTables 4 / DataLog
    Telemetry.log("Drive", driveMechanism);
  }
}
