package first.robot.opmodes;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

@Autonomous(name = "Drive Forward (Configurable)")
public class DriveForwardAutoOpMode extends PeriodicOpMode {
  private static final String DISTANCE_TOPIC = "/Dashboard/Auto/Drive Distance (m)";
  private static final Distance DEFAULT_DISTANCE = Meters.of(2.0);
  private static final double FORWARD_DRIVE_SPEED = 0.6;
  private static final Time PAUSE_DURATION = Seconds.of(0.5);
  private static final Distance BACKUP_DISTANCE = Meters.of(-0.5);
  private static final double BACKUP_DRIVE_SPEED = 0.4;

  private final Robot robot;
  private final DoubleEntry distanceEntry;
  private Command autoCommand;

  public DriveForwardAutoOpMode(Robot robot) {
    this.robot = robot;

    distanceEntry =
        NetworkTableInstance.getDefault()
            .getDoubleTopic(DISTANCE_TOPIC)
            .getEntry(DEFAULT_DISTANCE.in(Meters));

    distanceEntry.set(DEFAULT_DISTANCE.in(Meters));
  }

  @Override
  public void start() {
    autoCommand =
        robot.driveMechanism
            .run(
                coroutine -> {
                  Distance targetDistance = Meters.of(distanceEntry.get());

                  robot.driveMechanism.resetPose(Robot.INITIAL_POSE);

                  // Step 1: Drive the dashboard-specified distance
                  coroutine.await(
                      robot.driveMechanism.driveDistanceCommand(
                          targetDistance, FORWARD_DRIVE_SPEED));

                  // Step 2: Pause briefly
                  coroutine.wait(PAUSE_DURATION);

                  // Step 3: Back up
                  coroutine.await(
                      robot.driveMechanism.driveDistanceCommand(
                          BACKUP_DISTANCE, BACKUP_DRIVE_SPEED));
                })
            .named("Auto.DriveForward");

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
