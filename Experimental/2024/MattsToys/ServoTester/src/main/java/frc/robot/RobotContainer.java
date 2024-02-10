// Copyright (c) Matt Healy, Quasics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ServoPositionerCmd;
import frc.robot.subsystems.ServoHost;

public class RobotContainer {
  ServoHost m_servoHost = new ServoHost();
  XboxController m_controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_servoHost.setDefaultCommand(new ServoPositionerCmd(m_servoHost, () -> m_controller.getLeftY()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
