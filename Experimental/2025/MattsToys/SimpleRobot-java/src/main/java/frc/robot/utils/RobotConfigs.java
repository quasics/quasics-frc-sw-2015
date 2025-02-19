// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class RobotConfigs {
  public static record Position(Distance x, Distance y, Distance z) {
  }

  /**
   * Describes a camera's orientiation relative to the robot.
   * 
   * @oaran pitch downward pitch of camera (negative == upward)
   * @param roll left/right rotation of the camera
   * @param yaw
   */
  public static record Orientation(Angle pitch, Angle roll, Angle yaw) {
  }

  public static record Imaging(int imageWidth, int imageHeight, Angle fov, double fps) {
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

  // TODO: Add other data, such as PID settings for different things, etc.
  public static record RobotConfig(CameraConfig camera) {
  }

  public enum Robot {
    Simulation
  }

  static private final Map<Robot, RobotConfig> m_map = Collections.unmodifiableMap(createMap());

  public static RobotConfig getConfig(Robot robot) {
    return m_map.get(robot);
  }

  static private Map<Robot, RobotConfig> createMap() {
    var map = new HashMap<Robot, RobotConfig>();
    map.put(Robot.Simulation, new RobotConfig(
        new CameraConfig(
            "USBCamera1",
            // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
            // pose (which is considered to be its center of rotation at the floor level, or
            // Z = 0)...
            new Position(Meters.of(0.1), Meters.of(0.0), Meters.of(0.5)),
            // ...pitched 15 degrees up, pointing straightforward and in plane with the
            // robot,...
            new Orientation(Degrees.of(-15), Degrees.of(0), Degrees.of(0)),
            // ...with image dimensions 960x720, 100 degree field of view, and 30 FPS.
            new Imaging(960, 720, Degrees.of(10), 30))));
    return map;
  }
}
