package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/**
 * Simple vision subsystem interface.
 *
 * Allows clients to determine what targets are seen and basic information, but
 * doesn't do position identification.
 */
public interface IVision extends ISubsystem {
  static String SUBSYSTEM_NAME = "Vision";

  /**
   * @param id fiducial ID for the target
   * @param angle yaw to the angle (positive left)
   */
  record TargetData(int id, Angle angle, Distance distance) {
  }

  /////////////////////////////////////////////////////////////////////////////
  // Abstract methods
  //

  boolean hasTargetsInView();

  /**
   * @param robotPose current position of the robot on the field (e.g., from odometery)
   */
  List<TargetData> getVisibleTargets(Pose2d robotPose);

  /////////////////////////////////////////////////////////////////////////////
  // default methods and static helpers
  //

  static AprilTagFieldLayout loadLayout(String resourcePath) {
    // Load the layout of the AprilTags on the field.
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(resourcePath);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + resourcePath + ")");
      ioe.printStackTrace();
    }
    return tagLayout;
  }

  /**
   * Camera data set.
   *
   * @param camera      connection to the camera
   * @param transform3d defines the conversion from the robot's position, to the
   *                    cameras's
   * @param estimator   pose estimator associated with this camera. Note that (per
   *                    docs) the estimated poses can have a lot of
   *                    uncertainty/error baked into them when you are further
   *                    away from the targets.
   */
  public record CameraData(
      PhotonCamera camera, Transform3d transform3d, PhotonPoseEstimator estimator) {
  }
}
