// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.interfaces.IDrivebase;


/** Add your docs here. */
public class PathPlannerHelper {
  private SendableChooser<Command> m_chooser;

  public PathPlannerHelper(IDrivebase drivebase) {
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
      e.printStackTrace();
    }

    AutoBuilder.configure(drivebase::getOdometryPose, drivebase::resetOdometry,
        drivebase::getSpeed,
        (speeds, feedforwards)
            -> drivebase.driveWithPid(speeds),
        new PPLTVController(0.02), config, () -> {
          System.out.println("Autos: " + AutoBuilder.getAllAutoNames());
          System.out.println(
              "Is AutoBuilder Configured: " + AutoBuilder.isConfigured());
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, drivebase.asSubsystem());
    m_chooser = AutoBuilder.buildAutoChooser();
    // TODO: call setDefaultOption, setOption and add in non-path-planner
    // options.
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Get the auto command which is selected in our dropdown
   *
   * @return the auto command by the queried name
   */
  public Command getAuto() {
    System.out.println("Auto chooser" + m_chooser.getSelected());
    return m_chooser.getSelected();
  }

  public static Command doNothingAtHub(IDrivebase drivebase) {
    drivebase.resetOdometry(
        new Pose2d(new Translation2d(3.879, 3.942), new Rotation2d(0)));
    return Commands.print("Just sit there");
  }
}
