// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Example")
public class ExampleAuto implements OpMode {
  public ExampleAuto(Robot robot) {
    robot.exampleMechanism.exampleCondition.whileTrue(robot.exampleMechanism.exampleCommand());
  }
}
