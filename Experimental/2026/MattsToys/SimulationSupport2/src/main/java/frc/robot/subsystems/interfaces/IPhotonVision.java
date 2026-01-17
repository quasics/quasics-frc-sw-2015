// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.interfaces.IVision.TargetData;
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
 * Interface for explicitly PhotonVision-based vision subsystems.
 *
 * Note that this is *separate* from IVision, and thus can be used as a
 * "mix-in".
 */
public interface IPhotonVision {
  /**
   * Camera data set.
   *
   * @param camera      connection to the camera
   * @param transform3d defines the conversion from the robot's position, to the
   *                    cameras's
   * @param estimator   pose estimator associated with this camera. Note that
   *                    (per
   *                    docs) the estimated poses can have a lot of
   *                    uncertainty/error baked into them when you are further
   *                    away from the targets.
   */
  public record CameraData(PhotonCamera camera, Transform3d transform3d,
      PhotonPoseEstimator estimator) {}

  /**
   * Returns the list of CameraData records being used by this object.
   *
   * Note: this is exposed as public only for use with SimVisionWrapper.
   *
   * @return a list of CameraData objects
   */
  List<CameraData> getCameraDataForSimulation();

  /**
   * The AprilTagFieldLayout being used by this object.
   *
   * Note: this is exposed as public only for use with SimVisionWrapper.
   *
   * @return an AprilTagFieldLayout
   */
  AprilTagFieldLayout getFieldLayoutForSimulation();

  //
  // Field layout constants and helper functions.
  //

  /**
   * If true, use the Reefscape layout from 2025; if false, use the Crescendo
   * layout from 2024.
   */
  static final boolean USE_REEFSCAPE_LAYOUT = true;

  /**
   * If true, use the AndyMark configuration for the Reefscape layout; if false,
   * use the "welded" configuration.
   */
  static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;

  /** The field layout to use for vision processing/emulation. */
  static final AprilTagFields FIELD_LAYOUT = USE_REEFSCAPE_LAYOUT
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE
                ? AprilTagFields.k2025ReefscapeAndyMark
                : AprilTagFields.k2025ReefscapeWelded)
      : AprilTagFields.k2024Crescendo // Fall back on the 2024 game
      ;

  /**
   * Helper method to load a field layout.
   *
   * @param resourcePath path for the layout
   * @return the loaded field layout (or null on errors)
   */
  static AprilTagFieldLayout loadLayout(String resourcePath) {
    // Load the layout of the AprilTags on the field.
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(resourcePath);
    } catch (IOException ioe) {
      System.err.println(
          "Warning: failed to load April Tags layout (" + resourcePath + ")");
      ioe.printStackTrace();
    }
    return tagLayout;
  }

  //
  // Possibly obsolete functions (carried in from 2025)
  //

  // Making this a helper function, since getLatestResult() is now deprecated,
  // and I'm trying to cut down on the number of warnings.
  static PhotonPipelineResult getLatestResultsWrapper_deprecated(
      CameraData cameraData) {
    return cameraData.camera().getLatestResult();
  }

  /**
   * Returns estimated relative positioning data for all visible targets (if
   * any).
   *
   * @param cameraData  camera supplying the tracking data
   * @param fieldLayout field layout, used to determine fixed (absolute)
   *                    positions
   *                    for targets in
   *                    view
   * @param robotPose   robot's estimated position (used to compute relative
   *                    positioning for targets)
   * @return estimated relative positioning data for all visible targets
   */
  static List<TargetData> getTargetDataForCamera_deprecated(
      CameraData cameraData, AprilTagFieldLayout fieldLayout,
      Pose2d robotPose) {
    final var latestResults = getLatestResultsWrapper_deprecated(cameraData);
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
      final Distance distanceToTarget = Meters.of(
          PhotonUtils.getDistanceToPose(robotPose, tagPose.get().toPose2d()));

      TargetData curTargetData = new TargetData(
          result.fiducialId, Degrees.of(result.yaw), distanceToTarget);
      targets.add(curTargetData);
    }

    return targets;
  }
}
