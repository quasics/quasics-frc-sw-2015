// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.PathPlannerHelper;

public final class Autos {

  public static Command moveForward(IDrivebase drivebase) {
    return PathPlannerHelper.getAutonomousCommand(drivebase, "MoveForward1");
  }

  public static Command doNothingAtHub(IDrivebase drivebase) {
    drivebase.resetOdometry(new Pose2d(new Translation2d(3.879, 3.942), new Rotation2d(0)));
    return Commands.print("Just sit there");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
