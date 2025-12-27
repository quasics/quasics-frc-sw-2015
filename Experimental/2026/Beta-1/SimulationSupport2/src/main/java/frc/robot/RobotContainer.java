// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;

public class RobotContainer {
  IDrivebase drivebase = new Drivebase();
  private final Joystick m_driveController = new Joystick(OperatorConstants.DRIVER_JOYSTICK_ID);

  public RobotContainer() {
    drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(drivebase, this::getArcadeForward, this::getArcadeRotation));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Double getArcadeForward() {
    return m_driveController.getRawAxis(LogitechConstants.Dualshock.LeftYAxis);
  }

  public Double getArcadeRotation() {
    return m_driveController.getRawAxis(LogitechConstants.Dualshock.LeftXAxis);
  }
}