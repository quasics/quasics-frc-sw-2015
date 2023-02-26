// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;
import java.util.HashMap;
import java.util.Map;

/**
 * Collection of robot characteristics for various Quasics robots.
 *
 * <p>* Nike - our 2017 robot * Mae - 2020/2021 robot * Sally - 2022 robot * Gladys - 2023 robot *
 * Romi
 */
public class RobotSettingsLibrary {

  public enum Robot {
    Nike,
    Mae,
    Sally,
    Gladys,
    Romi;
  }

  private static final Map<Robot, RobotSettings> m_settingsMap =
      new HashMap<Robot, RobotSettings>();

  private static final double INCHES_PER_METER = 39.3701;

  private interface GearRatios {
    public static final double NIKE = 10.71;
    public static final double MAE = 10.71;
    public static final double SALLY = 8.45;
    public static final double GLADYS = 8.45;
    public static final double ROMI = 1;
  }

  /** Track widths for various robots (in meters). */
  private interface TrackWidths {
    public static final double GLADYS = 22.0 / INCHES_PER_METER;
    public static final double SALLY = 22.0 / INCHES_PER_METER;
    public static final double MAE =
        47.134344149315914763 / INCHES_PER_METER; // from in-code constants
    public static final double NIKE = 22.0 / INCHES_PER_METER;
    public static final double ROMI = 0.165;
  }

  static {
    m_settingsMap.put(
        Robot.Gladys,
        new RobotSettings(
            "Gladys", // robotName
            TrackWidths.GLADYS,
            GearRatios.GLADYS,
            // TODO(mjh) Calibrate Gladys' values for kS, kV, and kA (DriveProfileData)
            (DriveProfileData) null,
            // TODO(mjh) Calibrate Gladys' values for PID control (PIDConfig)
            (PIDConfig) null,
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1 // pigeonCanID
            ));
    m_settingsMap.put(
        Robot.Sally,
        new RobotSettings(
            "Sally", // robotName
            TrackWidths.SALLY,
            GearRatios.SALLY,
            // TODO(mjh) Calibrate Sally's values for kS, kV, and kA
            new DriveProfileData(0.31, 2.74, 0.249),
            // TODO(mjh) Calibrate Sally's values for PID control
            new PIDConfig(2.28, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1 // pigeonCanID
            ));
    m_settingsMap.put(
        Robot.Mae,
        new RobotSettings(
            "Mae", // robotName
            TrackWidths.MAE,
            GearRatios.MAE,
            // Drive configuration constants (computed 01Mar2022 w/ SysId)
            new DriveProfileData(/* kS= */ 0.13895, /* kV= */ 1.3143, /* kA= */ 0.1935),
            // PID control constants (computed 01Mar2022 w/ SysId)
            new PIDConfig(0.0011379, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1 // pigeonCanID
            ));
    m_settingsMap.put(
        Robot.Nike,
        new RobotSettings(
            "Nike", // robotName
            TrackWidths.NIKE,
            GearRatios.NIKE,
            // Drive configuration constants (computed 03Mar2022 w/ SysId)
            new DriveProfileData(/* kS= */ 0.14961, /* kV= */ 1.3717, /* kA= */ 0.1627),
            new PIDConfig(2.5682, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.Pigeon2,
            1 // pigeonCanID
            ));
    m_settingsMap.put(
        Robot.Romi,
        new RobotSettings(
            "Romi", // robotName
            TrackWidths.ROMI,
            GearRatios.ROMI,
            // TODO(mjh): Recalibrate Romi's values for kS, kV, and kA (if SysId ever
            // supports this) - these are from 2021
            new DriveProfileData(1.25, 5.7, 0.0176),
            // TODO(mjh): Recalibrate Romi's values for PID control (if SysId ever
            // supports this) - these are from 2021
            new PIDConfig(0.00352, 0, 0),
            // Note: Romi docs indicate that it's the right motor that's inverted, but I run
            // the Romi in reverse because the USB camera is mounted on my upper deck that
            // way.
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.Romi,
            0 // pigeonCanID
            ));
  }

  public RobotSettingsLibrary() {}

  public static RobotSettings getSettingsForRobot(Robot robot) {
    return m_settingsMap.get(robot);
  }

  /** Returns the robot settings for Sally (2023 robot). */
  public static RobotSettings getSettingsForGladys() {
    return m_settingsMap.get(Robot.Gladys);
  }

  /** Returns the robot settings for Sally (2022 robot). */
  public static RobotSettings getSettingsForSally() {
    return m_settingsMap.get(Robot.Sally);
  }

  /** Returns the robot settings for Mae (2021 robot). */
  public static RobotSettings getSettingsForMae() {
    return m_settingsMap.get(Robot.Mae);
  }

  /** Returns the robot settings for Nike (2019 robot). */
  public static RobotSettings getSettingsForNike() {
    return m_settingsMap.get(Robot.Nike);
  }

  /** Returns the robot settings for use on a Romi. */
  public static RobotSettings getSettingsForRomi() {
    return m_settingsMap.get(Robot.Romi);
  }
}
