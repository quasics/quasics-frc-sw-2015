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
 * <ul>
 *   <li>Nike - our 2017 robot
 *   <li>Mae - 2020/2021 robot
 *   <li>Sally - 2022 robot
 *   <li>Gladys - 2023 robot
 *   <li>Romi (used for training)
 *   <li>MattsRomi (used for training)
 * </ul>
 */
public class RobotSettingsLibrary {

  public static final int INVALID_PWM_PORT = -1;
  public static final int LED_PWM_PORT = 0;

  public interface LedLengths {
    public static final int TEST_STRIP_LENGTH = 14;
    public static final int GLADYS_STRIP_LENGTH = 14;
    public static final int ROMI_STRIP_LENGTH = -1;
  }

  public enum Robot {
    Nike,
    Mae,
    Sally,
    Gladys,
    Romi,
    MattsRomi;
  }

  private static final Map<Robot, RobotSettings> m_settingsMap =
      new HashMap<Robot, RobotSettings>();

  private static final double INCHES_PER_METER = 39.3701;

  /** Diameters of different wheels (in meters). */
  private interface WheelDiameters {
    public static final double ROMI = 0.07; // 70 mm / 2.75591 inches
    public static final double ANDYMARK_6IN_PLACTION = 6.0 / INCHES_PER_METER;
  }

  private interface GearRatios {
    public static final double NIKE = 10.71;
    public static final double MAE = 10.71;
    public static final double SALLY = 8.45;
    public static final double GLADYS = 8.45;
    public static final double ROMI = 1440.0; // Based on sample code
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
            WheelDiameters.ANDYMARK_6IN_PLACTION,
            // Drive base motor CAN IDs
            1,
            2,
            3,
            4,
            // TODO(mjh) Calibrate Gladys' values for kS, kV, and kA (DriveProfileData)
            new DriveProfileData(0.31, 2.74, 0.249),
            // TODO(mjh) Calibrate Gladys' values for PID control (PIDConfig)
            new PIDConfig(2.28, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1, // pigeonCanID
            LED_PWM_PORT,
            LedLengths.GLADYS_STRIP_LENGTH));
    m_settingsMap.put(
        Robot.Sally,
        new RobotSettings(
            "Sally", // robotName
            TrackWidths.SALLY,
            GearRatios.SALLY,
            WheelDiameters.ANDYMARK_6IN_PLACTION,
            // Drive base motor CAN IDs
            1,
            2,
            3,
            4,
            // TODO(mjh) Calibrate Sally's values for kS, kV, and kA
            new DriveProfileData(0.31, 2.74, 0.249),
            // TODO(mjh) Calibrate Sally's values for PID control
            new PIDConfig(2.28, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1, // pigeonCanID
            LED_PWM_PORT,
            LedLengths.TEST_STRIP_LENGTH));
    m_settingsMap.put(
        Robot.Mae,
        new RobotSettings(
            "Mae", // robotName
            TrackWidths.MAE,
            GearRatios.MAE,
            WheelDiameters.ANDYMARK_6IN_PLACTION,
            // Drive base motor CAN IDs
            1,
            2,
            3,
            4,
            // Drive configuration constants (computed 01Mar2022 w/ SysId)
            new DriveProfileData(/* kS= */ 0.13895, /* kV= */ 1.3143, /* kA= */ 0.1935),
            // PID control constants (computed 01Mar2022 w/ SysId)
            new PIDConfig(0.0011379, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.ADXRS450,
            1, // pigeonCanID
            LED_PWM_PORT,
            LedLengths.TEST_STRIP_LENGTH));
    m_settingsMap.put(
        Robot.Nike,
        new RobotSettings(
            "Nike", // robotName
            TrackWidths.NIKE,
            GearRatios.NIKE,
            WheelDiameters.ANDYMARK_6IN_PLACTION,
            // Drive base motor CAN IDs
            1,
            2,
            3,
            4,
            // Drive configuration constants (computed 03Mar2022 w/ SysId)
            new DriveProfileData(/* kS= */ 0.14961, /* kV= */ 1.3717, /* kA= */ 0.1627),
            new PIDConfig(2.5682, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.Pigeon2,
            1, // pigeonCanID
            LED_PWM_PORT,
            LedLengths.TEST_STRIP_LENGTH));
    m_settingsMap.put(
        Robot.Romi,
        new RobotSettings(
            "Romi", // robotName
            TrackWidths.ROMI,
            GearRatios.ROMI,
            WheelDiameters.ROMI,
            // Drive base motor IDs (not used for Romi)
            -1,
            -1,
            -1,
            -1,
            // TODO(mjh): Recalibrate Romi's values for kS, kV, and kA (if SysId ever
            // supports this) - these are from 2021
            new DriveProfileData(1.25, 5.7, 0.0176),
            // TODO(mjh): Recalibrate Romi's values for PID control (if SysId ever
            // supports this) - these are from 2021
            new PIDConfig(0.00352, 0, 0),
            RobotSettings.DriveMotorInversion.Left,
            RobotSettings.GyroType.Romi,
            -1, // pigeonCanID
            INVALID_PWM_PORT,
            LedLengths.ROMI_STRIP_LENGTH));
    m_settingsMap.put(
        Robot.MattsRomi,
        new RobotSettings(
            "MattsRomi", // robotName
            TrackWidths.ROMI,
            GearRatios.ROMI,
            WheelDiameters.ROMI,
            // Drive base motor IDs (not used for Romi)
            -1,
            -1,
            -1,
            -1,
            // TODO(mjh): Recalibrate Romi's values for kS, kV, and kA (if SysId ever
            // supports this) - these are from 2021
            new DriveProfileData(1.25, 5.7, 0.0176),
            // TODO(mjh): Recalibrate Romi's values for PID control (if SysId ever
            // supports this) - these are from 2021
            new PIDConfig(0.00352, 0, 0),
            // Note: Matt runs the Romi in reverse because the USB camera is mounted on
            // his unit's upper deck facing that backwards.
            RobotSettings.DriveMotorInversion.Right,
            RobotSettings.GyroType.Romi,
            -1, // pigeonCanID
            INVALID_PWM_PORT,
            LedLengths.ROMI_STRIP_LENGTH));
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

  /** Returns the robot settings for use on a standard Romi. */
  public static RobotSettings getSettingsForRomi() {
    return m_settingsMap.get(Robot.Romi);
  }

  /** Returns the robot settings for use on Matt's Romi. */
  public static RobotSettings getSettingsForMattsRomi() {
    return m_settingsMap.get(Robot.MattsRomi);
  }
}
