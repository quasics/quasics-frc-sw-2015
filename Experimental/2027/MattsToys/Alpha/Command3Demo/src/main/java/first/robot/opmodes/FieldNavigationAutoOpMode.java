package first.robot.opmodes;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "Navigate Field Waypoints")
public class FieldNavigationAutoOpMode extends PeriodicOpMode {
  private static final Pose2d WAYPOINT_1 =
      new Pose2d(Meters.of(5.0), Meters.of(5.5), new Rotation2d(Degrees.of(45.0)));
  private static final Pose2d WAYPOINT_2 =
      new Pose2d(Meters.of(3.5), Meters.of(2.0), new Rotation2d(Degrees.of(180.0)));

  private final Robot robot;
  private Command autoCommand;

  public FieldNavigationAutoOpMode(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    autoCommand =
        robot.driveMechanism
            .run(
                coroutine -> {
                  coroutine.await(robot.driveMechanism.resetPoseCommand(Robot.INITIAL_POSE));

                  // Waypoint 1: Drive forward to (5.0m, 5.5m) facing 45 degrees
                  coroutine.await(robot.driveMechanism.driveToPoseCommand(WAYPOINT_1));

                  // Waypoint 2: Drive to (3.5m, 2.0m) (lies behind the robot -> automatically reverses!)
                  // and then aligns heading to 180 degrees
                  coroutine.await(robot.driveMechanism.driveToPoseCommand(WAYPOINT_2));
                })
            .named("Auto.NavigateFieldWaypoints");

    Scheduler.getDefault().schedule(autoCommand);
  }

  @Override
  public void end() {
    if (autoCommand != null) {
      Scheduler.getDefault().cancel(autoCommand);
      autoCommand = null;
    }
  }
}
