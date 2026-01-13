// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.RobotConfigs.NO_CANDLE;

import frc.robot.constants.robots.SimulationPorts;
import frc.robot.util.RobotConfigs.ArmConfig;
import frc.robot.util.RobotConfigs.CameraConfig;
import frc.robot.util.RobotConfigs.CandleConfig;
import frc.robot.util.RobotConfigs.DriveConfig;
import frc.robot.util.RobotConfigs.DriveFeedForwardConfig;
import frc.robot.util.RobotConfigs.ElevatorConfig;
import frc.robot.util.RobotConfigs.ElevatorFeedForwardConfig;
import frc.robot.util.RobotConfigs.Imaging;
import frc.robot.util.RobotConfigs.LightingConfig;
import frc.robot.util.RobotConfigs.Orientation;
import frc.robot.util.RobotConfigs.PIDConfig;
import frc.robot.util.RobotConfigs.Position;
import frc.robot.util.RobotConfigs.RobotConfig;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Library of predefined robot configurations.
 * 
 * @see frc.robot.util.RobotConfigs
 */
public final class RobotConfigLibrary {
  /** The supported robots. */
  public enum Robot {
    /** Simulation-only */
    Simulation,
    /** Simulation-only */
    SimulationWithTwoCameras,
    // /** "Naked" drivebase used by the coding sub-team */
    // Sally,
    // /** 2025 ("Reefscape") robot */
    // Amelia
  }

  /**
   * Returns the configuration for a specific robot.
   *
   * @param robot the targeted robot
   * @return the configuration associated with the targeted robot
   */
  public static RobotConfig getConfig(Robot robot) {
    return m_map.get(robot);
  }

  /** Stores the actual mapping of robot IDs to configurations. */
  static private final Map<Robot, RobotConfig> m_map = Collections.unmodifiableMap(createMap());

  /**
   * Helper function, used to construct the underlying map. (Java doesn't
   * support inline specification of Map data.)
   *
   * @return the mapping of robots to configurations to be exposed to clients
   */
  static private Map<Robot, RobotConfig> createMap() {
    var map = new HashMap<Robot, RobotConfig>();
    map.put(Robot.Simulation, generateSingleCameraSimulationConfig());
    map.put(Robot.SimulationWithTwoCameras, generateTwoCameraSimulationConfig());

    //
    // Sanity checks to make sure that we have entries for all known robots.

    // Note that assertions are disabled by default. :-(
    // See
    // https://docs.oracle.com/javase/8/docs/technotes/guides/language/assert.html.
    assert (map.size() == Robot.values().length)
        : "Configurations for one or more robots are missing!";

    // Back up the assertion with something that can't be disabled.
    if (map.size() != Robot.values().length) {
      final int numRobotsWithoutConfigs = Robot.values().length - map.size();
      throw new RuntimeException("Configurations are missing for " +
          numRobotsWithoutConfigs + " robot(s)!");
    }
    return map;
  }

  /**
   * Drive config shared by our simulated drive base hardware (in multiple robot
   * configurations).
   */
  private final static DriveConfig SIMULATED_DRIVE_BASE_CONFIG = new DriveConfig(
      Inches.of(3), // Wheel radius
      Meters.of(0.381 * 2), // Trackwidth
      8.0, // Gearing
      new PIDConfig(1.6662),
      new PIDConfig(1.6662),
      new DriveFeedForwardConfig(
          // Linear data
          Volts.of(0.014183), Volts.of(1.9804), 0.19169,
          // Angular data
          Volts.of(2.6332), 0.5226));

  private static RobotConfig generateSingleCameraSimulationConfig() {
    final var cameraConfig = new CameraConfig(
        "USBCamera1",
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the
        // robot pose (which is considered to be its center of rotation at the
        // floor level, or Z = 0)...
        new Position(Meters.of(0.1), // x
            Meters.of(0.0), // y
            Meters.of(0.5)), // z
        // ...pitched 15 degrees up, pointing straightforward and in plane with
        // the robot,...
        new Orientation(Degrees.of(-15), // pitch
            Degrees.of(0), // roll
            Degrees.of(0) // yaw
        ),
        // ...with image dimensions 960x720, 100 degree field of view, and 30
        // FPS.
        new Imaging(960, 720, Degrees.of(100), 30));

    final var elevatorConfig = new ElevatorConfig(new PIDConfig(10.0, 0, 1),
        new ElevatorFeedForwardConfig(0.01, 0.05, 0.20, 0)
    // Note: PID and FF values were calculated using
    // SysId routines under simulation. new
    // PIDConfig(0.16168, 0, 0), new
    // ElevatorFeedForwardConfig(0.0015558, 0.05, 1.3321,
    // 0.03958) end of calibrated data
    );

    final var armConfig = new ArmConfig(
        // Note: PID and FF values are based on the Reefscape code base as of
        // 15Mar2025.
        new PIDConfig(6.0, 0.00, 0.00), null);

    final var lightingConfig = new LightingConfig(SimulationPorts.PWM.LIGHTING_PORT, 80);

    final var candleConfig = new CandleConfig(RobotConfigs.INVALID_CAN_ID);

    return new RobotConfig(
        SIMULATED_DRIVE_BASE_CONFIG,
        Arrays.asList(new CameraConfig[] {
            cameraConfig,
        }),
        elevatorConfig, armConfig, lightingConfig,
        candleConfig);
  }

  private static RobotConfig generateTwoCameraSimulationConfig() {
    return new RobotConfig(SIMULATED_DRIVE_BASE_CONFIG,
        Arrays.asList(new CameraConfig[] {
            new CameraConfig("USBCamera1",
                // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
                // pose (which is considered to be its center of rotation at the floor level, or
                // Z = 0)...
                new Position(Meters.of(0.1), // x
                    Meters.of(0.0), // y
                    Meters.of(0.5)), // z
                // ...pitched 15 degrees up, pointing straightforward and in plane with the
                // robot,...
                new Orientation(Degrees.of(-15), // pitch
                    Degrees.of(0), // roll
                    Degrees.of(0) // yaw
                ),
                // ...with image dimensions 960x720, 100 degree field of view, and 30 FPS.
                new Imaging(960, 720, Degrees.of(100), 30)),
            new CameraConfig("USBCamera2",
                // Our 2nd camera is mounted 0.25 meters back and 1.0 meters up from the robot
                // pose (which is considered to be its center of rotation at the floor level, or
                // Z = 0)...
                new Position(Meters.of(-0.25), // x
                    Meters.of(0.0), // y
                    Meters.of(1.0)), // z
                // ...pitched 0 degrees up, pointing straight backward and in plane with the
                // robot,...
                new Orientation(Degrees.of(0), // roll
                    Degrees.of(0), // pitch
                    Degrees.of(180) // yaw
                ),
                // ...with image dimensions 960x720, 100 degree field of view, and 30 FPS.
                new Imaging(960, 720, Degrees.of(100), 30)),
        }),
        new ElevatorConfig(
            // Note: PID and FF values are arbitrary for simulation use.
            new PIDConfig(10.0, 0, 1), new ElevatorFeedForwardConfig(0.01, 0.05, 0.20, 0)),
        new ArmConfig(
            // Note: PID and FF values are based on the Reefscape code base as of 15Mar2025.
            new PIDConfig(6.0, 0.00, 0.00), null),
        new LightingConfig(SimulationPorts.PWM.LIGHTING_PORT, 80),
        NO_CANDLE);
  }
}
