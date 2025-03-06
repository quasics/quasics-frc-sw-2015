// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.abstracts.AbstractVision;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.CameraConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Vision processing implementation for a single/multiple cameras, using the
 * Photonvision libraries/server.
 */
public class MultiCameraVision extends AbstractVision {
  /** Entries for each of the cameras on the robot. */
  protected final List<CameraData> m_cameraData = new LinkedList<CameraData>();

  private List<EstimatedRobotPose> m_latestEstimatedPoses = Collections.emptyList();

  /**
   * Constructor.
   * 
   * @param config robot configuration, including camera data
   */
  public MultiCameraVision(RobotConfig config) {
    // Add each of the cameras to our known set.
    List<RobotConfigs.CameraConfig> cameras = config.cameras();
    for (var camera : cameras) {
      addCameraToSet(camera);
    }
  }

  /**
   * Adds a camera to the known set.
   * 
   * Note: must be invoked *after* the tag layout has been loaded.
   * 
   * @param cameraConfig the configuratin for the camera
   */
  private void addCameraToSet(CameraConfig cameraConfig) {
    if (m_tagLayout == null) {
      throw new IllegalStateException("Can't add cameras without a known tag layout");
    }

    final PhotonCamera camera = new PhotonCamera(cameraConfig.name());
    final Transform3d robotToCamera = new Transform3d(new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(),
        cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(),
            cameraConfig.orientation().pitch(), cameraConfig.orientation().yaw()));
    final PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        m_tagLayout, POSE_STRATEGY, robotToCamera);
    m_cameraData.add(new CameraData(camera, robotToCamera, estimator));
  }

  public int getNumCameras() {
    return m_cameraData.size();
  }

  protected List<CameraData> getCameraData() {
    return m_cameraData;
  }

  @Override
  public void periodic() {
    super.periodic();

    // Where does the drive base think we are?
    final var optDrivePose = BulletinBoard.common.getValue(IDrivebase.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Update camera-specific estimators (and gather their results).
    double lastTimestamp = 0;
    List<EstimatedRobotPose> estimates = new LinkedList<EstimatedRobotPose>();
    for (CameraData cameraData : m_cameraData) {
      var estimate = updateEstimateForCamera(
          cameraData.camera(), cameraData.estimator(),
          drivePose);
      if (!estimate.isEmpty()) {
        var estimatedPose = estimate.get();
        estimates.add(estimatedPose);
        lastTimestamp = Math.max(estimatedPose.timestampSeconds, lastTimestamp);
      }
    }

    // Save it, and publish it.
    m_latestEstimatedPoses = Collections.unmodifiableList(estimates);
    publishDataToBulletinBoard(!estimates.isEmpty(), lastTimestamp, m_latestEstimatedPoses);
  }

  @Override
  public List<EstimatedRobotPose> getEstimatedPoses() {
    return m_latestEstimatedPoses;
  }
}
