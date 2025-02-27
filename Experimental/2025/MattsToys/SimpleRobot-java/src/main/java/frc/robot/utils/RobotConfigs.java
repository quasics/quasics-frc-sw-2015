// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.simulations.SimulationPorts;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Defines various configuration data for robot subsystems.
 */
public class RobotConfigs {
  // TODO: Add definitions for actual hardware.
  /** The supported robots. */
  public enum Robot {
    /** Simulation-only */
    Simulation,
    /** "Naked" drivebase used by the coding sub-team */
    Sally,
    /** 2025 ("Reefscape") robot */
    Amelia
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
   * Location of something in terms of the robot's center (in terms of the robot
   * coordinate system).
   *
   * This is currently used for the camera(s), but could easily be used for other
   * things as needed. (Note: I could've used a Translation3D for this, but felt
   * that "position" was more readable.)
   *
   * @param x distance along the X axis (front (+)/back (-)) from center
   * @param y distance along the Y axis (left (+)/right (-)) from center
   * @param z distance along the Z axis (up (+)/down (-)) from center
   *
   * @see <a
   *      href=
   *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system">Robot
   *      coordinate system</a>
   */
  public static record Position(Distance x, Distance y, Distance z) {
  }

  /**
   * Describes a camera's orientiation relative to the robot.
   *
   * @param roll  The counterclockwise rotation angle around the X axis (roll).
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
   * @param yaw   The counterclockwise rotation angle around the Z axis (yaw).
   */
  public static record Orientation(Angle pitch, Angle roll, Angle yaw) {
  }

  /**
   * Defines the image-related characteristics of a camera on the robot.
   *
   * Note that some of these characteristics would only be used (directly) in the
   * code for simulation purposes.
   *
   * @param width  camera field width (in pixels)
   * @param height camera field height (in pixels)
   * @param fov    field of view (e.g., 100 degrees)
   * @param fps    frames per second produced by the video stream
   */
  public static record Imaging(int width, int height, Angle fov, double fps) {
  }

  /**
   * Describes the camera's configuration.
   *
   * @param name        name of the camera (as exposed through PhotonVision)
   * @param pos         camera position, relative to the center of the robot
   * @param orientation angling/rotation of the camera (relative to robot
   *                    centerline, flat)
   * @param imaging     characteristics of the camera's image feed
   */
  public static record CameraConfig(
      String name, Position pos, Orientation orientation, Imaging imaging) {
  }

  /**
   * PID configuration settings.
   *
   * @param kP proportional constant
   * @param kI integral constant
   * @param kD derivitive constant
   */
  public static record PIDConfig(double kP, double kI, double kD) {
    /**
     * Overloaded ctor for kP-only configs.
     *
     * @param kP proportional constant
     */
    public PIDConfig(double kP) {
      this(kP, 0.0, 0.0);
    }
  }

  /**
   * Elevator Feed forward settings.
   *
   * TODO: Convert kV/kA from raw doubles to unit-based values.
   *
   * @param kS static gain
   * @param kG gravity gain
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public static record ElevatorFeedForwardConfig(Voltage kS, Voltage kG, double kV, double kA) {
    /**
     * Overloaded constructor.
     *
     * @param kS static gain, in V
     * @param kG gravity gain, in V
     * @param kV kV, in V/(m/s)
     * @param kA kA, in V/(m/s^2)
     */
    public ElevatorFeedForwardConfig(double kS, double kG, double kV, double kA) {
      this(Volts.of(kS), Volts.of(kG), kV, kA);
    }
  }

  /**
   * Configuration data for an elevator.
   *
   * @param pid         PID configuration settings for the elevator's motors
   * @param feedForward feedforward data for the elevator
   */
  public static record ElevatorConfig(PIDConfig pid, ElevatorFeedForwardConfig feedForward) {
  }

  /**
   * Simple (i.e., not for elevators) feedforward data.
   *
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public static record SimpleFeedForwardConfig(Voltage kV, double kA) {
    /**
     * Overloaded constructor.
     *
     * @param kV kV, in V/(m/s); must be > 0
     * @param kA kA, in V/(m/s^2)
     */
    public SimpleFeedForwardConfig(double kV, double kA) {
      this(Volts.of(kV), kA);
    }
  }

  /**
   * Drive Feed forward settings.
   *
   * @param linear  linear feedforward settings
   * @param angular angular (rotational) feedforward settings
   */
  public static record DriveFeedForwardConfig(
      SimpleFeedForwardConfig linear, SimpleFeedForwardConfig angular) {
    /**
     * Overloaded constructor, taking pairs of (kV, kA) as discrete values
     *
     * @param kvLinear  linear feedforward kV value
     * @param kaLinear  linear feedforward kA value
     * @param kvAngular angular (rotational) feedforward kV value
     * @param kaAngular angular (rotational) feedforward kA value
     */
    public DriveFeedForwardConfig(
        Voltage kvLinear, double kaLinear, Voltage kvAngular, double kaAngular) {
      this(new SimpleFeedForwardConfig(kvLinear, kaLinear),
          new SimpleFeedForwardConfig(kvAngular, kaAngular));
    }
  }

  /**
   * Drive base configuration data.
   *
   * @param wheelRadius radius of the drive base wheels
   * @param trackWidth  maximum width between drive base wheels
   * @param pid         PID configuration for the drivebase
   * @param feedForward feedforward configuration for the drivebase
   */
  public static record DriveConfig(Distance wheelRadius, Distance trackWidth,
      double gearing, // (gearing between motor and wheel axel (>=1))
      PIDConfig pid, DriveFeedForwardConfig feedForward) {
  }

  /**
   * Lighting subsystem configuration data.
   *
   * @param pwmPort     the PWM port driving the LED strip
   * @param stripLength the length (in pixels/cells) of the LED strip
   */
  public static record LightingConfig(int pwmPort, int stripLength) {
  }

  /**
   * Collective robot configuration data.
   *
   * @param drive    drive base configuration (may be null)
   * @param camera   camera configuration (may be null)
   * @param elevator elevator configuration (may be null)
   * @param lighting lighting configuration (may be null)
   */
  public static record RobotConfig(
      DriveConfig drive, CameraConfig camera, ElevatorConfig elevator, LightingConfig lighting) {
    /** @return true iff the configuration includes data for the drivebase */
    public boolean hasDrive() {
      return drive != null;
    }

    /** @return true iff the configuration includes data for the camera */
    public boolean hasCamera() {
      return camera != null;
    }

    /** @return true iff the configuration includes data for the elevator */
    public boolean hasElevator() {
      return elevator != null;
    }

    /**
     * @return true iff the configuration includes data for the lighting subsystem
     */
    public boolean hasLighting() {
      return lighting != null;
    }
  }

  /**
   * Helper function, used to construct the underlying map. (Java doesn't support
   * inline specification of Map data.)
   *
   * @return the mapping of robots to configurations to be exposed to clients
   */
  static private Map<Robot, RobotConfig> createMap() {
    @SuppressWarnings("unused")
    final DriveConfig NO_DRIVE = null;
    final CameraConfig NO_CAMERA = null;
    final ElevatorConfig NO_ELEVATOR = null;
    final LightingConfig NO_LIGHTING = null;

    var map = new HashMap<Robot, RobotConfig>();
    map.put(Robot.Simulation,
        new RobotConfig(
            new DriveConfig(Inches.of(3), // Wheel radius
                Units.Meters.of(0.381 * 2), // Trackwidth
                8.0, // Gearing
                new PIDConfig(1.6018),
                new DriveFeedForwardConfig(
                    // ksLinear: 0.014183
                    Volts.of(1.9802), 0.19202, // Linear data
                    // ksAngular: 0.011388
                    Volts.of(1.5001), 0.29782) // Angular data
            ),
            new CameraConfig("USBCamera1",
                // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
                // pose (which is considered to be its center of rotation at the floor level, or
                // Z = 0)...
                new Position(Meters.of(0.1), // x
                    Meters.of(0.0), // y
                    Meters.of(0.5)), // z
                // ...pitched 15 degrees up, pointing straightforward and in plane with the
                // robot,...
                new Orientation(Degrees.of(0), // roll
                    Degrees.of(-15), // pitch
                    Degrees.of(0) // yaw
                ),
                // ...with image dimensions 960x720, 100 degree field of view, and 30 FPS.
                new Imaging(960, 720, Degrees.of(100), 30)),
            new ElevatorConfig(
                // Note: PID and FF values are arbitrary for simulation use.
                new PIDConfig(10.0, 0, 0),
                new ElevatorFeedForwardConfig(0.01, 0.05, 0.20, 0)),
            new LightingConfig(SimulationPorts.LIGHTING_PWM_ID, 80)));

    map.put(Robot.Sally,
        new RobotConfig(
            // TODO: Update DriveConfig data to match Sally's configuration.
            new DriveConfig(Inches.of(3), // Wheel radius
                Meters.of(0.5588) /* 22 in (from 2024) */,
                8.45, // Gearing (from 2024)
                new PIDConfig(0.29613), // (from 2024)
                new DriveFeedForwardConfig(Volts.of(0.19529), 0.01, // Linear data (from 2024)
                    Volts.of(0.19529), 0.01) // Angular data (FAKE)
            ),
            NO_CAMERA, NO_ELEVATOR, NO_LIGHTING));

    map.put(Robot.Amelia,
        // TODO: Add subsystem configurations for Amelia
        new RobotConfig(
            // TODO: Update DriveConfig data to match Amelia's configuration.
            new DriveConfig(Inches.of(3), // Wheel radius
                Meters.of(0.5588) /* 22 in (from 2024) */,
                8.45, // Gearing (from 2024)
                new PIDConfig(0.29613), // (from 2024 robot)
                new DriveFeedForwardConfig(Volts.of(0.18), 0.0, // Linear data (from 2024)
                    Volts.of(0.18), 0.0) // Angular data (FAKE)
            ),
            NO_CAMERA,
            NO_ELEVATOR,
            NO_LIGHTING));

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
      throw new RuntimeException(
          "Configurations are missing for " + numRobotsWithoutConfigs + " robot(s)!");
    }
    return map;
  }
}
