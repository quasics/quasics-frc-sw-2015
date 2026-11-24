package first.robot.opmodes;

import static org.wpilib.units.Units.Meters;

import first.robot.Robot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.units.measure.Distance;

@Teleop(name = "Tank Drive Teleop")
public class TankTeleopOpMode extends PeriodicOpMode {
  private static final Distance TEST_DRIVE_DISTANCE = Meters.of(1.5);
  private static final double TEST_DRIVE_SPEED = 0.5;
  private static final Pose2d TEST_TARGET_POSE =
      new Pose2d(Meters.of(4.0), Meters.of(5.0), Rotation2d.k180deg);

  public TankTeleopOpMode(Robot robot) {
    // Drive straight 1.5m at 50% power (Xbox A / Keyboard 'Z')
    robot.driveDistanceTrigger.onTrue(
        robot.driveMechanism.driveDistanceCommand(TEST_DRIVE_DISTANCE, TEST_DRIVE_SPEED));

    // Reset robot pose back to starting coordinates (Xbox B / Keyboard 'X')
    robot.resetPoseTrigger.onTrue(robot.driveMechanism.resetPoseCommand(Robot.INITIAL_POSE));

    // Navigate to (4.0m, 5.0m) facing 180 degrees (Xbox X / Keyboard 'C')
    robot.driveToPoseTrigger.onTrue(robot.driveMechanism.driveToPoseCommand(TEST_TARGET_POSE));
  }
}
