// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RainbowLighting;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.BrokenCanDrivebase;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.LightingInterface;
import frc.robot.subsystems.SimulationDrivebase;

public class RobotContainer {
  final static int LIGHTING_PWM_PORT = 9;
  final static int NUM_LIGHTS = 40;

  private final XboxController m_controller = new XboxController(0);
  private final LightingInterface m_lighting = new Lighting(LIGHTING_PWM_PORT, NUM_LIGHTS);
  private final AbstractDrivebase m_drivebase;

  public RobotContainer() {
    if (Robot.isReal()) {
      m_drivebase = new BrokenCanDrivebase();
    } else {
      m_drivebase = new SimulationDrivebase();
    }
    configureBindings();
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase, m_controller));
    m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
