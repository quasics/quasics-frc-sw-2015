package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import first.robot.mechanisms.DriveMechanism;
import first.robot.mechanisms.SimulatedDriveMechanism;
import java.util.Arrays;
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
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StringEntry;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

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
  private static final String START_PRESET_TOPIC = "Preset";
  private static final String START_AVAILABLE_PRESETS_TOPIC =
      "Available Presets";
  private static final String START_X_TOPIC = "X (m)";
  private static final String START_Y_TOPIC = "Y (m)";
  private static final String START_HEADING_TOPIC = "Heading (deg)";

  /** Preset field starting locations (Blue-alliance NWU field coordinates). */
  public enum StartingLocation {
    LEFT(Meters.of(1.5), Meters.of(6.0), Degrees.of(0.0)),
    CENTER(Meters.of(1.5), Meters.of(4.0), Degrees.of(0.0)),
    RIGHT(Meters.of(1.5), Meters.of(2.0), Degrees.of(0.0)),
    CUSTOM(Meters.of(2.0), Meters.of(4.0), Degrees.of(0.0));

    private final Pose2d pose;

    StartingLocation(Distance x, Distance y, Angle heading) {
      this.pose = new Pose2d(x, y, new Rotation2d(heading));
    }

    public Pose2d toPose2d() {
      return pose;
    }

    public static StartingLocation fromNameOrCustom(String name) {
      if (name == null) {
        return CUSTOM;
      }
      for (StartingLocation location : values()) {
        if (location.name().equalsIgnoreCase(name.trim())) {
          return location;
        }
      }
      return CUSTOM;
    }
  }

  public static final StartingLocation DEFAULT_STARTING_LOCATION =
      StartingLocation.CENTER;
  public static final Pose2d DEFAULT_INITIAL_POSE =
      DEFAULT_STARTING_LOCATION.toPose2d();

  public final DriveMechanism driveMechanism;
  public final Trigger driveDistanceTrigger;
  public final Trigger resetPoseTrigger;
  public final Trigger driveToPoseTrigger;

  private final StringEntry startPresetEntry;
  private final StringArrayPublisher availablePresetsPublisher;
  private final DoubleEntry startXEntry;
  private final DoubleEntry startYEntry;
  private final DoubleEntry startHeadingEntry;

  private StartingLocation lastAppliedPreset = DEFAULT_STARTING_LOCATION;
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

    // Publish preset selector and editable X/Y/Heading entries for the drive
    // team
    NetworkTable startPoseTable =
        NetworkTableInstance.getDefault().getTable(START_POSE_TABLE);
    startPresetEntry = startPoseTable.getStringTopic(START_PRESET_TOPIC)
                           .getEntry(DEFAULT_STARTING_LOCATION.name());
    availablePresetsPublisher =
        startPoseTable.getStringArrayTopic(START_AVAILABLE_PRESETS_TOPIC)
            .publish();

    startXEntry = startPoseTable.getDoubleTopic(START_X_TOPIC)
                      .getEntry(DEFAULT_INITIAL_POSE.getX());
    startYEntry = startPoseTable.getDoubleTopic(START_Y_TOPIC)
                      .getEntry(DEFAULT_INITIAL_POSE.getY());
    startHeadingEntry =
        startPoseTable.getDoubleTopic(START_HEADING_TOPIC)
            .getEntry(DEFAULT_INITIAL_POSE.getRotation().getDegrees());

    availablePresetsPublisher.set(Arrays.stream(StartingLocation.values())
            .map(Enum::name)
            .toArray(String[] ::new));
    startPresetEntry.set(DEFAULT_STARTING_LOCATION.name());
    writePoseToEntries(DEFAULT_INITIAL_POSE);

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

  private void writePoseToEntries(Pose2d pose) {
    startXEntry.set(pose.getX());
    startYEntry.set(pose.getY());
    startHeadingEntry.set(pose.getRotation().getDegrees());
  }

  private void syncPreMatchStartPose() {
    StartingLocation selectedPreset =
        StartingLocation.fromNameOrCustom(startPresetEntry.get());

    // 1. If the drive team selected a new non-CUSTOM preset, populate
    // X/Y/Heading from that preset
    if (selectedPreset != lastAppliedPreset) {
      lastAppliedPreset = selectedPreset;
      if (selectedPreset != StartingLocation.CUSTOM) {
        Pose2d presetPose = selectedPreset.toPose2d();
        writePoseToEntries(presetPose);
        lastAppliedStartPose = presetPose;
        driveMechanism.syncInitialPose(presetPose);
        return;
      }
    }

    // 2. Otherwise check if X/Y/Heading were manually edited on the dashboard
    Pose2d configuredPose = getConfiguredStartPose();
    if (!configuredPose.equals(lastAppliedStartPose)) {
      lastAppliedStartPose = configuredPose;
      if (selectedPreset != StartingLocation.CUSTOM
          && !configuredPose.equals(selectedPreset.toPose2d())) {
        lastAppliedPreset = StartingLocation.CUSTOM;
        startPresetEntry.set(StartingLocation.CUSTOM.name());
      }
      driveMechanism.syncInitialPose(configuredPose);
    }
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

    // Reflect preset selections or custom X/Y/Heading edits immediately on
    // Field2d while not enabled
    if (!RobotState.isEnabled()) {
      syncPreMatchStartPose();
    }

    // Runs active OpMode coroutines, triggers, odometry updates, and simulation
    // physics
    Scheduler.getDefault().run();

    // Flush mechanism telemetry to NetworkTables 4 / DataLog
    Telemetry.log("Drive", driveMechanism);
  }
}
