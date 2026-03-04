// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.abstracts.AbstractDrivebase;
import frc.robot.subsystems.interfaces.drivebase.IDrivebase;
import frc.robot.subsystems.interfaces.drivebase.IDrivebasePlus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPLTVController;


/** Add your docs here. */
public class PathPlannerHelper {

  public PathPlannerHelper() throws IllegalAccessException {
    throw new IllegalAccessException("Don't do that!! :(");
  } 

  public static Command getAutonomousCommand(AbstractDrivebase abstractDrivebase) {
    com.pathplanner.lib.config.RobotConfig config = null;
    try{
      config = com.pathplanner.lib.config.RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println("Hey! Listen!");
      e.printStackTrace();
    }

    AutoBuilder.configure(
      abstractDrivebase::getPose,
      abstractDrivebase::resetPose,
      abstractDrivebase::getCurrentSpeeds,
      (speeds, feedforwards) -> abstractDrivebase.driveWithPid(speeds),
      new PPLTVController(0.02),
      config,
      () -> {
         var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
      },
      abstractDrivebase
    );
    return new PathPlannerAuto("MoveForward1");
  }

}
