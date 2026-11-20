package first.robot;

import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

@Autonomous(name = "Simple Drive Auto", group = "Drive")
public class SimpleDriveAuto implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  public SimpleDriveAuto(Robot robot) {
    enabled.whileTrue(robot.driveBase.driveForward(.25));
  }
  
}
