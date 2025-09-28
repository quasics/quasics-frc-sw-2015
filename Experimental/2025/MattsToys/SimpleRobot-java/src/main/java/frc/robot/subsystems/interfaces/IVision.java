package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
   * @param angle yaw to the angle (negative values means that it's to left of camera center)
   */
  record TargetData(int id, Angle angle, Distance distance) {}

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
  public record
      CameraData(PhotonCamera camera, Transform3d transform3d, PhotonPoseEstimator estimator) {}

  /////////////////////////////////////////////////////////////////////////////
  // Abstract methods
  //

  boolean hasTargetsInView();

  /**
   * @param robotPose current position of the robot on the field (e.g., from odometery)
   */
  List<TargetData> getVisibleTargets(Pose2d robotPose);

  /**
   * Returns the list of CameraData records being used by this object.
   *
   * Note: this is exposed as public only for use with SimVisionWrapper.
   *
   * @return a list of CameraData objects
   */
  List<CameraData> getCameraDataForSimulation();

  /**
   * Returns the most recent pose estimates, based on camera data.
   *
   * @return the most recent pose estimates
   */
  List<Pose2d> getEstimatedPoses();

  /**
   * The AprilTagFieldLayout being used by this object.
   *
   * Note: this is exposed as public only for use with SimVisionWrapper.
   *
   * @return an AprilTagFieldLayout
   */
  AprilTagFieldLayout getFieldLayoutForSimulation();

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

  // Making this a helper function, since getLatestResult() is now deprecated, and
  // I'm trying to cut down on the number of warnings.
  static PhotonPipelineResult getLatestResultsWrapper(CameraData cameraData) {
    // TODO: look at replacing this with something in a reusable base class to try to
    // cache data, to handle the deprecation.
    return cameraData.camera().getLatestResult();
  }

  static List<TargetData> getTargetDataForCamera(
      CameraData cameraData, AprilTagFieldLayout fieldLayout, Pose2d robotPose) {
    final var latestResults = IVision.getLatestResultsWrapper(cameraData);
    if (!latestResults.hasTargets()) {
      return Collections.emptyList();
    }

    List<TargetData> targets = new LinkedList<TargetData>();
    for (PhotonTrackedTarget result : latestResults.targets) {
      var tagPose = fieldLayout.getTagPose(result.fiducialId);
      if (tagPose.isEmpty()) {
        continue;
      }

      // Given where we *know* the target is on the field, and where we *think*
      // that the robot is, how far away are we from the target?
      final Distance distanceToTarget =
          Meters.of(PhotonUtils.getDistanceToPose(robotPose, tagPose.get().toPose2d()));

      TargetData curTargetData =
          new TargetData(result.fiducialId, Degrees.of(result.yaw), distanceToTarget);
      targets.add(curTargetData);
    }

    return targets;
  }

  /** Trivial implementation of IVision (e.g., if we don't have a camera). */
  public class NullVision implements IVision {
    /** Constructor. */
    public NullVision() {
      System.out.println("INFO: allocating NullVision");
    }

    @Override
    public List<CameraData> getCameraDataForSimulation() {
      return Collections.emptyList();
    }

    @Override
    public boolean hasTargetsInView() {
      return false;
    }

    @Override
    public List<TargetData> getVisibleTargets(Pose2d robotPose) {
      return Collections.emptyList();
    }

    @Override
    public AprilTagFieldLayout getFieldLayoutForSimulation() {
      return null;
    }

    @Override
    public List<Pose2d> getEstimatedPoses() {
      return Collections.emptyList();
    }
  }
}
