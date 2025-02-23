// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/**
 * Defines various configuration data for robot subsystems.
 */
public class RobotConfigs {
  // TODO: Add definitions for actual hardware.
  public enum Robot {
    Simulation
  }

  /** @return the configuration associated with a specific robot */
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
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system
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
  public static record CameraConfig(String name, Position pos, Orientation orientation,
      Imaging imaging) {
  }

  public static record PIDConfig(double kP, double kI, double kD) {
    /** Overloaded ctor for kP-only configs. */
    public PIDConfig(double kP) {
      this(kP, 0.0, 0.0);
    }
  }

  /**
   * Elevator Feed forward settings.
   * 
   * TODO: Convert kV/kA from raw doubles to unit-based values.
   * 
   * @param kS static gain, in V
   * @param kG gravity gain, in V (only used for elevator)
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public static record ElevatorFeedForwardConfig(Voltage kS, Voltage kG, double kV, double kA) {
    public ElevatorFeedForwardConfig(double kS, double kG, double kV, double kA) {
      this(Volts.of(kS), Volts.of(kG), kV, kA);
    }
  }

  public static record ElevatorConfig(PIDConfig pid, ElevatorFeedForwardConfig feedForward) {
  }

  /**
   * Simple feedforward data.
   * 
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public static record SimpleFeedForwardConfig(Voltage kV, double kA) {
    public SimpleFeedForwardConfig(double kV, double kA) {
      this(Volts.of(kV), kA);
    }
  }

  /**
   * Drive Feed forward settings.
   */
  public static record DriveFeedForwardConfig(SimpleFeedForwardConfig linear, SimpleFeedForwardConfig angular) {
    public DriveFeedForwardConfig(Voltage kvLinear, double kaLinear, Voltage kvAngular, double kaAngular) {
      this(new SimpleFeedForwardConfig(kvLinear, kaLinear), new SimpleFeedForwardConfig(kvAngular, kaAngular));
    }
  }

  /**
   * Drive base configuration data.
   */
  public static record DriveConfig(
      Distance wheelRadius,
      Distance trackWidth,
      double gearing, // (gearing between motor and wheel axel (>=1))
      PIDConfig pid,
      DriveFeedForwardConfig feedForward) {
  }

  /** Collective robot configuration data. */
  public static record RobotConfig(DriveConfig drive, CameraConfig camera, ElevatorConfig elevator) {
  }

  /**
   * Helper function, used to construct the underlying map. (Java doesn't support
   * inline specification of Map data.)
   */
  static private Map<Robot, RobotConfig> createMap() {
    var map = new HashMap<Robot, RobotConfig>();
    map.put(Robot.Simulation, new RobotConfig(
        new DriveConfig(
            Inches.of(3), // Wheel radius
            Units.Meters.of(0.381 * 2), // Trackwidth
            8.0, // Gearing
            new PIDConfig(
                1.6018),
            new DriveFeedForwardConfig(
                // ksLinear: 0.014183
                Volts.of(1.9802), 0.19202, // Linear data
                // ksAngular: 0.011388
                Volts.of(1.5001), 0.29782) // Angular data
        ),
        new CameraConfig(
            "USBCamera1",
            // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
            // pose (which is considered to be its center of rotation at the floor level, or
            // Z = 0)...
            new Position(
                Meters.of(0.1), // x
                Meters.of(0.0), // y
                Meters.of(0.5)), // z
            // ...pitched 15 degrees up, pointing straightforward and in plane with the
            // robot,...
            new Orientation(
                Degrees.of(0), // roll
                Degrees.of(-15), // pitch
                Degrees.of(0) // yaw
            ),
            // ...with image dimensions 960x720, 100 degree field of view, and 30 FPS.
            new Imaging(960, 720, Degrees.of(100), 30)),
        new ElevatorConfig(
            // Note: PID and FF values are arbitrary for simulation use.
            new PIDConfig(10.0, 0, 0),
            new ElevatorFeedForwardConfig(0.01, 0.05, 0.20, 0))));

    // Sanity check to make sure that we have entries for all known robots.
    assert (map.size() == Robot.values().length) : "Configurations for one or more robots are missing!";
    return map;
  }
}
