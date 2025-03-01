// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.abstracts.AbstractVision;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Vision processing implementation for a single camera, based on Photonvision.
 */
public class SingleCameraVision extends AbstractVision {
  /** Data associated with our (single) camera. */
  protected final CameraData m_cameraData;

  /** Timestamp of the last pose estimation update. */
  private double m_lastEstTimestamp = 0;

  private Optional<EstimatedRobotPose> m_lastEstimatedPose = Optional.empty();

  /**
   * Constructs a Vision subsystem, based on a specified robot configuration.
   *
   * @param config robot configuration
   */
  public SingleCameraVision(RobotConfig config) {
    this(config.cameras().get(0));
  }

  /**
   * Constructs a Vision subsystem, based on a specified robot configuration.
   *
   * @param cameraConfig robot configuration
   */
  private SingleCameraVision(RobotConfigs.CameraConfig cameraConfig) {
    this(cameraConfig.name(),
        new Transform3d(new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(),
            cameraConfig.pos().z()),
            new Rotation3d(cameraConfig.orientation().roll(),
                cameraConfig.orientation().pitch(), cameraConfig.orientation().yaw())));
  }

  /**
   * Delegated constructor for a Vision subystem.
   *
   * @param cameraName             name configured for the camera in the
   *                               CameraServer
   * @param robotToCameraTransform Transform3d from the center of the robot to the
   *                               camera mount position (ie, robot ➔ camera) in
   *                               the Robot Coordinate System.
   *
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system
   */
  private SingleCameraVision(String cameraName, Transform3d robotToCameraTransform) {
    // Connect to our camera. (May be a simulation.)
    PhotonCamera camera = null;
    if (cameraName != null && !cameraName.isBlank()) {
      camera = new PhotonCamera(cameraName);
    }

    //
    // Set up the vision pose estimator
    m_cameraData = new CameraData(
        camera,
        robotToCameraTransform,
        new PhotonPoseEstimator(
            m_tagLayout, POSE_STRATEGY, robotToCameraTransform));

    // Configure what to do in a multi-tag environment (like Crescendo) when only
    // one tag can be seen.
    m_cameraData.estimator().setMultiTagFallbackStrategy(POSE_STRATEGY);
  }

  public int getNumCameras() {
    return 1;
  }

  protected List<CameraData> getCameraData() {
    return Collections.singletonList(m_cameraData);
  }

  /**
   * Updates/caches the latest estimated robot pose on the field from vision data,
   * which may be empty. This should only be called once per loop, and will be
   * invoked from our <code>periodic()</code> method.
   * 
   * TODO: Look at uniting functionality from this (and multi-camera) in base
   * class.
   *
   * @see #periodic()
   */
  private void updateEstimatedGlobalPose() {
    PhotonPoseEstimator estimator = m_cameraData.estimator();
    PhotonCamera camera = m_cameraData.camera();
    if (camera == null || estimator == null) {
      // Nothing to do.
      return;
    }

    // Update the vision pose estimator with the latest robot pose from the drive
    // base.
    BulletinBoard.common.getValue(IDrivebase.ODOMETRY_KEY, Pose2d.class).ifPresentOrElse(pose -> {
      Pose2d pose2d = (Pose2d) pose;
      estimator.setLastPose(pose2d);
      estimator.setReferencePose(pose2d);
    }, () -> System.err.println("Warning: no robot drive pose available."));

    // Update the pose estimator with the latest vision measurements.
    List<PhotonPipelineResult> results = m_cameraData.camera().getAllUnreadResults();
    if (results.isEmpty()) {
      // No results? Nothing to do.
      return;
    }

    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    double lastEstimatedTimestamp = 0;
    for (PhotonPipelineResult photonPipelineResult : results) {
      lastEstimatedPose = estimator.update(photonPipelineResult);
      lastEstimatedTimestamp = photonPipelineResult.getTimestampSeconds();
    }

    // Update "recently updated" and "last" values.
    boolean recentlyUpdated = Math
        .abs(lastEstimatedTimestamp - m_lastEstTimestamp) > TIMESTAMP_RECENCY_THRESHOLD_SECS;
    if (recentlyUpdated && lastEstimatedPose.isPresent()) {
      m_lastEstTimestamp = lastEstimatedTimestamp;
      m_lastEstimatedPose = lastEstimatedPose;
    } else {
      m_lastEstimatedPose = Optional.empty();
    }

    // Update published data
    publishDataToBulletinBoard(recentlyUpdated, lastEstimatedTimestamp, lastEstimatedPose);
  }

  private static void publishDataToBulletinBoard(
      boolean recentlyUpdated,
      double lastTimestamp,
      Optional<EstimatedRobotPose> lastPose) {
    if (recentlyUpdated && lastPose.isPresent()) {
      BulletinBoard.common.updateValue(POSE_TIMESTAMP_KEY, lastTimestamp);
      BulletinBoard.common.updateValue(
          POSES_KEY,
          Collections.singletonList(lastPose.get()));
    } else {
      BulletinBoard.common.clearValue(POSE_TIMESTAMP_KEY);
      BulletinBoard.common.clearValue(POSES_KEY);
    }
  }

  // Note: this method will be called once per scheduler run
  @Override
  public void periodic() {
    super.periodic();

    updateEstimatedGlobalPose();
  }

  @Override
  public List<EstimatedRobotPose> getEstimatedPoses() {
    if (m_lastEstimatedPose.isEmpty()) {
      return Collections.emptyList();
    }

    return Collections.singletonList(m_lastEstimatedPose.get());
  }
}
