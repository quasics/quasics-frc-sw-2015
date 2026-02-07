// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.RobotConfigs.NO_ARM;
import static frc.robot.util.RobotConfigs.NO_CAMERA;
import static frc.robot.util.RobotConfigs.NO_CANDLE;
import static frc.robot.util.RobotConfigs.NO_ELEVATOR;
import static frc.robot.util.RobotConfigs.NO_LIGHTING;
import static frc.robot.util.RobotConfigs.NO_POWER_DISTRIBUTOR;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.constants.robots.SimulationPorts;
import frc.robot.util.RobotConfigs.ArmConfig;
import frc.robot.util.RobotConfigs.CameraConfig;
import frc.robot.util.RobotConfigs.CandleConfig;
import frc.robot.util.RobotConfigs.DriveConfig;
import frc.robot.util.RobotConfigs.DriveFeedForwardConfig;
import frc.robot.util.RobotConfigs.DriveOrientation;
import frc.robot.util.RobotConfigs.DriveType;
import frc.robot.util.RobotConfigs.ElevatorConfig;
import frc.robot.util.RobotConfigs.ElevatorFeedForwardConfig;
import frc.robot.util.RobotConfigs.Imaging;
import frc.robot.util.RobotConfigs.LightingConfig;
import frc.robot.util.RobotConfigs.Orientation;
import frc.robot.util.RobotConfigs.PIDConfig;
import frc.robot.util.RobotConfigs.Position;
import frc.robot.util.RobotConfigs.PowerDistributor;
import frc.robot.util.RobotConfigs.RobotConfig;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
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
    /** "Naked" drivebase used by the coding sub-team */
    Sally,
    // /** 2025 ("Reefscape") robot */
    // Amelia,
    // /** 2026 ("Rebuilt") robot */
    Rebuilt2026, // 20.25" trackwidth (roughly)
  }

  //
  // Static data members
  //

  /**
   * Drive config shared by our simulated drive base hardware (in multiple robot
   * configurations).
   */
  private final static DriveConfig SIMULATED_DRIVE_BASE_CONFIG = new DriveConfig(DriveType.Simulated,
      // Wheel radius
      Inches.of(3),
      Meters.of(0.381 * 2), // Trackwidth
      8.0, // Gearing
      DriveOrientation.RightInverted, new PIDConfig(1.6662),
      new PIDConfig(2.5662),
      new DriveFeedForwardConfig(
          // Linear data
          Volts.of(0.011404), Volts.of(2.2002), 0.56983,
          // Angular data
          Volts.of(3.8643), 1.7422));
  /**
   * Pre-change:
   * --------------
   * kP: 1.6403
   * kS: 0.01417
   * kV: 1.9803
   * kA: 0.19186
   * 
   * kS (Ang): 0.021613
   * kV (Ang): 2.6333
   * kA (Ang): 0.52163
   * 
   * Post-change:
   * kP: 2.5662
   * kS: 0.011404
   * kV: 2.2002
   * kA: 0.56983
   * 
   * kS (Ang): 0.019793
   * kV (Ang): 3.8643
   * kA (Ang): 1.7422
   */

  /**
   * Stores the actual mapping of robot IDs to configurations.
   *
   * Note that this must come after any other data members, in order to ensure
   * correct ordering of construction.
   */
  static private final Map<Robot, RobotConfig> m_map = Collections.unmodifiableMap(createMap());

  //
  // Static methods
  //

  /**
   * Returns the configuration for a specific robot.
   *
   * @param robot the targeted robot
   * @return the configuration associated with the targeted robot
   */
  public static RobotConfig getConfig(Robot robot) {
    return m_map.get(robot);
  }

  /**
   * Helper function, used to construct the underlying map. (Java doesn't
   * support inline specification of Map data.)
   *
   * @return the mapping of robots to configurations to be exposed to clients
   */
  static private Map<Robot, RobotConfig> createMap() {
    var map = new HashMap<Robot, RobotConfig>();
    map.put(Robot.Simulation, generateSingleCameraSimulationConfig());
    map.put(
        Robot.SimulationWithTwoCameras, generateTwoCameraSimulationConfig());
    map.put(Robot.Sally, generateSallyConfig());
    map.put(Robot.Rebuilt2026, generate2026Config());

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
      throw new RuntimeException("Configurations are missing for "
          + numRobotsWithoutConfigs + " robot(s)!");
    }

    return map;
  }

  private static RobotConfig generateSingleCameraSimulationConfig() {
    final var cameraConfig = new CameraConfig("USBCamera1",
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

    final int lightingSideViewSize = 10;
    final List<Integer> lightingSubviews = new LinkedList<Integer>();
    // Note that if a CANdle is enabled under simulation, that will inject a
    // subview for that in position 0 during setup.
    lightingSubviews.add(lightingSideViewSize); // left side
    lightingSubviews.add(lightingSideViewSize); // right side
    final var lightingConfig = new LightingConfig(
        SimulationPorts.PWM.LIGHTING_PORT, 80, lightingSubviews);

    final var candleConfig = new CandleConfig(RobotConfigs.INVALID_CAN_ID);

    return new RobotConfig(true, SIMULATED_DRIVE_BASE_CONFIG,
        Arrays.asList(new CameraConfig[] {
            cameraConfig,
        }),
        elevatorConfig, armConfig, lightingConfig, candleConfig,
        NO_POWER_DISTRIBUTOR);
  }

  private static RobotConfig generateTwoCameraSimulationConfig() {
    return new RobotConfig(true, SIMULATED_DRIVE_BASE_CONFIG,
        Arrays.asList(new CameraConfig[] {
            new CameraConfig("USBCamera1",
                // Our camera is mounted 0.1 meters forward and 0.5
                // meters up
                // from the robot pose (which is considered to be its
                // center of
                // rotation at the floor level, or Z = 0)...
                new Position(Meters.of(0.1), // x
                    Meters.of(0.0), // y
                    Meters.of(0.5)), // z
                // ...pitched 15 degrees up, pointing straightforward
                // and in
                // plane with the robot,...
                new Orientation(Degrees.of(-15), // pitch
                    Degrees.of(0), // roll
                    Degrees.of(0) // yaw
                ),
                // ...with image dimensions 960x720, 100 degree field of
                // view,
                // and 30 FPS.
                new Imaging(960, 720, Degrees.of(100), 30)),
            new CameraConfig("USBCamera2",
                // Our 2nd camera is mounted 0.25 meters back and 1.0
                // meters up
                // from the robot pose (which is considered to be its
                // center of
                // rotation at the floor level, or Z = 0)...
                new Position(Meters.of(-0.25), // x
                    Meters.of(0.0), // y
                    Meters.of(1.0)), // z
                // ...pitched 0 degrees up, pointing straight backward
                // and in
                // plane with the robot,...
                new Orientation(Degrees.of(0), // roll
                    Degrees.of(0), // pitch
                    Degrees.of(180) // yaw
                ),
                // ...with image dimensions 960x720, 100 degree field of
                // view,
                // and 30 FPS.
                new Imaging(960, 720, Degrees.of(100), 30)),
        }),
        new ElevatorConfig(
            // Note: PID and FF values are arbitrary for simulation use.
            new PIDConfig(10.0, 0, 1),
            new ElevatorFeedForwardConfig(0.01, 0.05, 0.20, 0)),
        new ArmConfig(
            // Note: PID and FF values are based on the Reefscape code base as
            // of 15Mar2025.
            new PIDConfig(6.0, 0.00, 0.00), null),
        new LightingConfig(SimulationPorts.PWM.LIGHTING_PORT, 80), NO_CANDLE,
        NO_POWER_DISTRIBUTOR);
  }

  private static RobotConfig generateSallyConfig() {
    PowerDistributor power = new PowerDistributor(ModuleType.kRev);

    return new RobotConfig(false,
        new DriveConfig(DriveType.CanSparkMax, Inches.of(3), // Wheel radius
            Meters.of(0.5588) /* 22 in (from 2024) */,
            8.45, // Gearing (from 2024),
            DriveOrientation.RightInverted,
            // TODO: Update DriveConfig data for Sally in 2026
            // configuration/profile, including independent left/right PID.
            new PIDConfig(0.29613), // Left PID (from 2024)
            new PIDConfig(0.29613), // Right PID (from 2024)
            // TODO: Add kS value for Sally's drivebase.
            new DriveFeedForwardConfig(Volts.of(0.19529),
                0.01, // Linear data (from 2024)
                Volts.of(0.19529), 0.01) // Angular data (FAKE)
        ),
        NO_CAMERA, NO_ELEVATOR, NO_ARM, NO_LIGHTING, NO_CANDLE, power);
  }

  private static RobotConfig generate2026Config() {
    // TODO: Update 2026 drive configuration data with real numbers.
    DriveConfig drive = new DriveConfig(DriveType.ThriftyNova, Inches.of(3),
        Inches.of(20.25), // Hand-wavy.....
        8.45, DriveOrientation.RightInverted,
        // TODO: Compute left/right PID and FF data for 2026 robot
        new PIDConfig(0.29613), // Left PID
        new PIDConfig(0.29613), // Right PID
        new DriveFeedForwardConfig(Volts.of(0), // Ks (linear)
            Volts.of(0.19529), // kV (Linear)
            0.01, // Ka (Linear)
            Volts.of(0.19529), 0.01) // Angular data (FAKE)
    );

    PowerDistributor power = new PowerDistributor(ModuleType.kRev);

    return new RobotConfig(false, drive, NO_CAMERA, NO_ELEVATOR, NO_ARM,
        NO_LIGHTING, NO_CANDLE, power);
  }
}
