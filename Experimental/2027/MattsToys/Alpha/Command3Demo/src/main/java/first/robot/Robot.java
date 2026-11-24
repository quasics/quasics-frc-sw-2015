package first.robot;

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

  public static final Pose2d INITIAL_POSE =
      new Pose2d(Meters.of(2.0), Meters.of(4.0), new Rotation2d());

  public final DriveMechanism driveMechanism;
  public final Trigger driveDistanceTrigger;
  public final Trigger resetPoseTrigger;
  public final Trigger driveToPoseTrigger;

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

    // Optional: enable simulated sensor noise before instantiating
    // SimulatedDriveMechanism SimulatedDriveMechanism.setNoiseEnabled(true);

    driveMechanism = RobotBase.isReal()
        ? new DriveMechanism(leftSpeedSupplier, rightSpeedSupplier)
        : new SimulatedDriveMechanism(leftSpeedSupplier, rightSpeedSupplier);

    // Place the robot at a visible starting pose on the Field2d map
    driveMechanism.resetPose(INITIAL_POSE);
  }

  @Override
  public void robotPeriodic() {
    // Cancel any running commands immediately upon entering Disabled mode
    boolean isDisabled = RobotState.isDisabled();
    if (isDisabled && !wasDisabled) {
      Scheduler.getDefault().cancelAll();
    }
    wasDisabled = isDisabled;

    // Runs active OpMode coroutines, triggers, odometry updates, and simulation
    // physics
    Scheduler.getDefault().run();

    // Flush mechanism telemetry to NetworkTables 4 / DataLog
    Telemetry.log("Drive", driveMechanism);
  }
}
