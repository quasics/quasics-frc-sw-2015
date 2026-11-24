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
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.Joystick;
import org.wpilib.driverstation.MatchState;
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

  // Field dimensions for Blue <-> Red alliance mirroring (update to match
  // season field layout)
  private static final Distance FIELD_LENGTH = Meters.of(16.54);
  private static final Distance FIELD_WIDTH = Meters.of(8.21);
  private static final boolean USE_ROTATIONAL_SYMMETRY = true;

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
  private static final String START_ALLIANCE_TOPIC = "Active Alliance";
  private static final String START_X_TOPIC = "X (m)";
  private static final String START_Y_TOPIC = "Y (m)";
  private static final String START_HEADING_TOPIC = "Heading (deg)";

  /**
   * Preset field starting locations defined in Blue-alliance NWU coordinates
   * (LEFT/CENTER/RIGHT are from the driver's perspective facing the field).
   */
  public enum StartingLocation {
    LEFT(Meters.of(1.5), Meters.of(6.0), Degrees.of(0.0)),
    CENTER(Meters.of(1.5), Meters.of(4.0), Degrees.of(0.0)),
    RIGHT(Meters.of(1.5), Meters.of(2.0), Degrees.of(0.0)),
    CUSTOM(Meters.of(2.0), Meters.of(4.0), Degrees.of(0.0));

    private final Pose2d bluePose;

    StartingLocation(Distance x, Distance y, Angle heading) {
      this.bluePose = new Pose2d(x, y, new Rotation2d(heading));
    }

    /** Returns this preset's pose mirrored for the given alliance color. */
    public Pose2d toPose2d(Alliance alliance) {
      return alliance == Alliance.RED ? mirrorBluePoseToRed(bluePose)
                                      : bluePose;
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

  public final DriveMechanism driveMechanism;
  public final Trigger driveDistanceTrigger;
  public final Trigger resetPoseTrigger;
  public final Trigger driveToPoseTrigger;

  private final StringEntry startPresetEntry;
  private final StringArrayPublisher availablePresetsPublisher;
  private final StringEntry activeAllianceEntry;
  private final DoubleEntry startXEntry;
  private final DoubleEntry startYEntry;
  private final DoubleEntry startHeadingEntry;

  // Alliance and starting pose state from the last robotPeriodic() cycle, used
  // to detect changes and update Field2d accordingly.
  private java.util.Optional<Alliance> lastFmsAlliance =
      java.util.Optional.empty();
  private Alliance lastAppliedAlliance;
  private StartingLocation lastAppliedPreset = DEFAULT_STARTING_LOCATION;
  private Pose2d lastAppliedStartPose;
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

    lastFmsAlliance = MatchState.getAlliance();
    lastAppliedAlliance = lastFmsAlliance.orElse(Alliance.BLUE);
    lastAppliedStartPose =
        DEFAULT_STARTING_LOCATION.toPose2d(lastAppliedAlliance);

    // Publish preset selector, active alliance indicator, and editable
    // X/Y/Heading entries
    NetworkTable startPoseTable =
        NetworkTableInstance.getDefault().getTable(START_POSE_TABLE);
    startPresetEntry = startPoseTable.getStringTopic(START_PRESET_TOPIC)
                           .getEntry(DEFAULT_STARTING_LOCATION.name());
    availablePresetsPublisher =
        startPoseTable.getStringArrayTopic(START_AVAILABLE_PRESETS_TOPIC)
            .publish();
    activeAllianceEntry = startPoseTable.getStringTopic(START_ALLIANCE_TOPIC)
                              .getEntry(lastAppliedAlliance.name());

    startXEntry = startPoseTable.getDoubleTopic(START_X_TOPIC)
                      .getEntry(lastAppliedStartPose.getX());
    startYEntry = startPoseTable.getDoubleTopic(START_Y_TOPIC)
                      .getEntry(lastAppliedStartPose.getY());
    startHeadingEntry =
        startPoseTable.getDoubleTopic(START_HEADING_TOPIC)
            .getEntry(lastAppliedStartPose.getRotation().getDegrees());

    availablePresetsPublisher.set(Arrays.stream(StartingLocation.values())
            .map(Enum::name)
            .toArray(String[] ::new));
    startPresetEntry.set(DEFAULT_STARTING_LOCATION.name());
    activeAllianceEntry.set(lastAppliedAlliance.name());
    writePoseToEntries(lastAppliedStartPose);

    // Optional: enable simulated sensor noise before instantiating
    // SimulatedDriveMechanism SimulatedDriveMechanism.setNoiseEnabled(true);

    driveMechanism = RobotBase.isReal()
        ? new DriveMechanism(leftSpeedSupplier, rightSpeedSupplier)
        : new SimulatedDriveMechanism(leftSpeedSupplier, rightSpeedSupplier);

    // Sync initial starting pose immediately on Field2d before any OpMode is
    // enabled
    driveMechanism.syncInitialPose(lastAppliedStartPose);
  }

  /**
   * Returns the active MatchState alliance, defaulting to BLUE if not yet
   * reported.
   */
  public static Alliance getCurrentAlliance() {
    return MatchState.getAlliance().orElse(Alliance.BLUE);
  }

  /**
   * Mirrors a Blue-origin field Pose2d onto the Red alliance side of the
   * field.
   */
  public static Pose2d mirrorBluePoseToRed(Pose2d bluePose) {
    Distance redX = FIELD_LENGTH.minus(bluePose.getMeasureX());
    if (USE_ROTATIONAL_SYMMETRY) {
      Distance redY = FIELD_WIDTH.minus(bluePose.getMeasureY());
      Rotation2d redHeading = bluePose.getRotation().plus(Rotation2d.k180deg);
      return new Pose2d(redX, redY, redHeading);
    } else {
      Rotation2d redHeading = Rotation2d.k180deg.minus(bluePose.getRotation());
      return new Pose2d(redX, bluePose.getMeasureY(), redHeading);
    }
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

  private static Alliance parseAllianceOrDefault(
      String name, Alliance fallback) {
    if (name == null) {
      return fallback;
    }
    String normalized = name.trim().toUpperCase();
    if (normalized.startsWith("RED")) {
      return Alliance.RED;
    }
    if (normalized.startsWith("BLUE")) {
      return Alliance.BLUE;
    }
    return fallback;
  }

  private void syncPreMatchStartPose() {
    // 1. Check if FMS / Sim GUI reported a new alliance (requires Sim GUI to be
    // in Disabled, not Disconnected)
    java.util.Optional<Alliance> fmsAlliance = MatchState.getAlliance();
    Alliance currentAlliance = lastAppliedAlliance;

    if (!fmsAlliance.equals(lastFmsAlliance)) {
      lastFmsAlliance = fmsAlliance;
      if (fmsAlliance.isPresent()) {
        currentAlliance = fmsAlliance.get();
        activeAllianceEntry.set(currentAlliance.name());
      }
    } else {
      // 2. Otherwise allow manual override via Dashboard/Start
      // Pose/Active Alliance in NT viewer
      Alliance dashboardAlliance = parseAllianceOrDefault(
          activeAllianceEntry.get(), lastAppliedAlliance);
      if (dashboardAlliance != lastAppliedAlliance) {
        currentAlliance = dashboardAlliance;
        activeAllianceEntry.set(currentAlliance.name());
      }
    }

    StartingLocation selectedPreset =
        StartingLocation.fromNameOrCustom(startPresetEntry.get());

    boolean allianceChanged = currentAlliance != lastAppliedAlliance;
    boolean presetChanged = selectedPreset != lastAppliedPreset;

    if (allianceChanged) {
      lastAppliedAlliance = currentAlliance;
    }

    // 3. Recompute X/Y/Heading whenever either the preset or the alliance color
    // changes
    if (presetChanged || allianceChanged) {
      lastAppliedPreset = selectedPreset;
      if (selectedPreset != StartingLocation.CUSTOM) {
        Pose2d presetPose = selectedPreset.toPose2d(currentAlliance);
        writePoseToEntries(presetPose);
        lastAppliedStartPose = presetPose;
        driveMechanism.syncInitialPose(presetPose);
        return;
      }
    }

    // 4. Otherwise check if X/Y/Heading were manually edited on the dashboard
    Pose2d configuredPose = getConfiguredStartPose();
    if (!configuredPose.equals(lastAppliedStartPose)) {
      lastAppliedStartPose = configuredPose;
      if (selectedPreset != StartingLocation.CUSTOM
          && !configuredPose.equals(selectedPreset.toPose2d(currentAlliance))) {
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

    // Reflect alliance changes, preset selections, or custom edits on Field2d
    // while not enabled
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
