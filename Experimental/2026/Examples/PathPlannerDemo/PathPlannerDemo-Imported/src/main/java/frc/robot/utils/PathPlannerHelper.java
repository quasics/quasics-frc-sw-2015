// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.abstracts.AbstractDrivebase;
import frc.robot.subsystems.interfaces.drivebase.IDrivebasePlus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;


/** Add your docs here. */
public class PathPlannerHelper {
private AbstractDrivebase m_abstractDrivebase;

  public PathPlannerHelper(AbstractDrivebase abstractDrivebase) {
    com.pathplanner.lib.config.RobotConfig config = null;
    try{
      config = com.pathplanner.lib.config.RobotConfig.fromGUISettings();
    } catch (Exception e) {
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
  } 
}
