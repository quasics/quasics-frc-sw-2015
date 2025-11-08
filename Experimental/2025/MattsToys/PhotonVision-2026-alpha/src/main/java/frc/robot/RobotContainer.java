// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.live.BetterVision;
import frc.robot.subsystems.simulations.CameraSimulator;
import frc.robot.utils.RobotConfigs;

/**
 * This class serves as the central hub for the declarative setup of our "command-based"
 * robot project, under the standard WPILib definition for this construct.
 * 
 * @see https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html#robotcontainer
 */
public class RobotContainer {
  /** Configuration settings for the hardware to be targeted. */
  final RobotConfigs.RobotConfig m_config = RobotConfigs.getConfig(RobotConfigs.Robot.Simulation);
  final CameraSimulator m_cameraSim = new CameraSimulator(m_config, new BetterVision(m_config));
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
