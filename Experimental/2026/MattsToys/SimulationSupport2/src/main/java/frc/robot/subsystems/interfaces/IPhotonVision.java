// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
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
}
