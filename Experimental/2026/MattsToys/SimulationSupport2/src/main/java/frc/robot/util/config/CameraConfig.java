package frc.robot.util.config;

/**
 * Describes the camera's configuration.
 *
 * @param name        name of the camera (as exposed through PhotonVision)
 * @param pos         camera position, relative to the center of the robot
 * @param orientation angling/rotation of the camera (relative to robot
 *                    centerline, flat)
 * @param imaging     characteristics of the camera's image feed
 */
public record CameraConfig(
    String name, Position pos, Orientation orientation, Imaging imaging) {
}