package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/**
 * Interface for an explicitly PhotonVision-based vision subsystem.
 */
public interface IPhotonVision extends IVision {
  /**
   * Camera data set.
   *
   * @param camera      connection to the camera
   * @param transform3d defines the conversion from the robot's position, to the
   *                    cameras's
   * @param estimator   pose estimator associated with this camera. Note that
   *     (per
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
}
