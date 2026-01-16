// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
      PhotonPoseEstimator estimator) {
  }

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
      System.err.println("Warning: failed to load April Tags layout (" +
          resourcePath + ")");
      ioe.printStackTrace();
    }
    return tagLayout;
  }

}
