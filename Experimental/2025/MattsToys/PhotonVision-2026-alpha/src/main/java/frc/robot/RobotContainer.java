// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.live.BetterVision;
import frc.robot.subsystems.simulations.CameraSimulator;
import frc.robot.utils.RobotConfigs;

public class RobotContainer {
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
