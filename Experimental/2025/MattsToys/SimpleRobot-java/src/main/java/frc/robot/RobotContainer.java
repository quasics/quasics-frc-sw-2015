// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveForDistance;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.simulations.SimDrivebase;

public class RobotContainer {
  // Subsystems
  Vision m_vision = new Vision();
  IDrivebase m_drivebase = new SimDrivebase();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new DriveForDistance(m_drivebase, .50, Meters.of(3));
    // return Commands.print("No autonomous command configured");
  }
}
