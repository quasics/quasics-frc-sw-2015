// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PathPlannerHelper {
  public PathPlannerHelper() throws IllegalAccessException {
    throw new IllegalAccessException("Don't do that!! :(");
  }

  // TODO: Make sure this only happens once
  public static boolean configureAutoBuilder(
      IDrivebase drivebase) {
    com.pathplanner.lib.config.RobotConfig config = null;
    try {
      config = com.pathplanner.lib.config.RobotConfig.fromGUISettings();
      System.out.println("Auto Config settings");
      System.out.println("Holonomic:" + config.isHolonomic);
      System.out.println("Mass (Kg):" + config.massKG);
      System.out.println("Moment of Inertia:" + config.MOI);
      System.out.println(
          "Wheel Radius:" + config.moduleConfig.wheelRadiusMeters);
      System.out.println("Gearing:" + config.moduleConfig.driveMotor);
      System.out.println(
          "Max Speed:" + config.moduleConfig.maxDriveVelocityMPS);
      System.out.println("Wheel COF:" + config.moduleConfig.wheelCOF);
      System.out.println("Drive Motor:" + config.moduleConfig.driveMotor);
      System.out.println(
          "Drive Current Limit:" + config.moduleConfig.driveCurrentLimit);

    } catch (Exception e) {
      System.out.println("Hey! Listen!");
      e.printStackTrace();
    }

    AutoBuilder.configure(drivebase::getOdometryPose,
        drivebase::resetOdometry, drivebase::getSpeed,
        (speeds, feedforwards) -> drivebase.driveWithPid(speeds),
        new PPLTVController(0.02), config, () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drivebase.asSubsystem());
    return true;
  }

  public static Command getAutonomousCommand(IDrivebase drivebase, String auto) {
    configureAutoBuilder(drivebase);
    return new PathPlannerAuto(auto);
  }

  public static Command autoChooser(IDrivebase drivebase) {
    System.out.println("AutoChooser is being called!");
    configureAutoBuilder(drivebase);
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // DifferentialDrive.m_rightMotor.setSafetyEnabled(false);
    return autoChooser.getSelected();
  }
}
