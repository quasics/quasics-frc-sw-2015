// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;

public class RobotContainer {
  IDrivebase drivebase = new Drivebase();

  public RobotContainer() {
    drivebase.asSubsystem()
        .setDefaultCommand(new ArcadeDrive(drivebase, this::getArcadeForward, this::getArcadeRotation));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Double getArcadeForward() {
    // TODO: Replace with real input
    return 0.0;
  }

  public Double getArcadeRotation() {
    // TODO: Replace with real input
    return 0.0;
  }
}