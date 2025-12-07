// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.drivebase.IDrivebasePlus;
import frc.robot.subsystems.interfaces.vision.IVision;
import frc.robot.subsystems.interfaces.vision.IVisionPlus;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.CameraConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Vision processing implementation for a single/multiple cameras, using the
 * Photonvision libraries/server.
 */
public class BetterVision extends SubsystemBase implements IVisionPlus {
  /** Pose strategy to be used for resolving estimates. */
  protected static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  protected final AprilTagFieldLayout m_tagLayout;

  private static final boolean USE_REEFSCAPE_LAYOUT = true;
  private static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;

  private static final AprilTagFields FIELD_LAYOUT = USE_REEFSCAPE_LAYOUT
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE ? AprilTagFields.k2025ReefscapeAndyMark
          : AprilTagFields.k2025ReefscapeWelded)
      : AprilTagFields.k2024Crescendo // Fall back on last year's game
  ;

  /** Entries for each of the cameras on the robot. */
  protected final List<CameraData> m_cameraData = new ArrayList<CameraData>();

  protected final Map<Integer, EstimatedRobotPose> m_estimatedPosesCache = new HashMap<Integer, EstimatedRobotPose>();

  /**
   * Estimated poses (if any) from the most recent camera data.
   *
   * TODO: Replace this with the data from the cache.
   */
  private List<EstimatedRobotPose> m_latestEstimatedPoses = Collections.emptyList();

  /**
   * Constructor.
   *
   * @param config robot configuration, including camera data
   */
  public BetterVision(RobotConfig config) {
    setName(IVisionPlus.SUBSYSTEM_NAME);
    m_tagLayout = IVision.loadLayout(FIELD_LAYOUT.m_resourceFile);

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
    final Transform3d robotToCamera = new Transform3d(
        new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(), cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(), cameraConfig.orientation().pitch(),
            cameraConfig.orientation().yaw()));
    final PhotonPoseEstimator estimator = new PhotonPoseEstimator(m_tagLayout, POSE_STRATEGY, robotToCamera);
    m_cameraData.add(new CameraData(camera, robotToCamera, estimator));
  }

  /**
   * Returns the number of cameras on the robot.
   *
   * @return the number of cameras on the robot
   */
  public int getNumCameras() {
    return m_cameraData.size();
  }

  /**
   * Returns the camera data for all cameras on the robot.
   *
   * @return the camera data for all cameras on the robot (unmodifiable)
   */
  protected List<CameraData> getCameraData() {
    return m_cameraData;
  }

  /**
   * Applies any updates for the estimator on a camera, optionally integrating the
   * last/reference pose provided by the drivebase.
   *
   * @param pipelineResult latest pipeline results from a camera
   * @param estimator      pose estimator being updated
   * @param drivePose      last reported pose from the drivebase (or null)
   * @return the updated estimate, based on the camera data
   */
  protected static Optional<EstimatedRobotPose> updateEstimateForCamera(
      final PhotonPipelineResult pipelineResult, final PhotonPoseEstimator estimator,
      final Pose2d drivePose) {
    if (pipelineResult == null) {
      return Optional.empty();
    }

    if (drivePose != null) {
      estimator.setLastPose(drivePose);
      estimator.setReferencePose(drivePose);
    }
    return estimator.update(pipelineResult);
  }

  /**
   * Publishes the latest pose data to the bulletin board from all cameras with
   * available data.
   *
   * @param recentlyUpdated true if the data was updated recently
   * @param lastTimestamp   the timestamp of the last pose update
   * @param lastPoses       the last computed poses (if any)
   */
  protected static void publishDataToBulletinBoard(
      boolean recentlyUpdated, double lastTimestamp, List<EstimatedRobotPose> lastPoses) {
    if (!recentlyUpdated || lastPoses == null || lastPoses.isEmpty()) {
      BulletinBoard.common.clearValue(POSE_TIMESTAMP_KEY);
      BulletinBoard.common.clearValue(POSES_KEY);
    } else {
      BulletinBoard.common.updateValue(POSE_TIMESTAMP_KEY, lastTimestamp);
      BulletinBoard.common.updateValue(POSES_KEY, lastPoses);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // IVision functions.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public final List<CameraData> getCameraDataForSimulation() {
    return getCameraData();
  }

  @Override
  public final AprilTagFieldLayout getFieldLayoutForSimulation() {
    return m_tagLayout;
  }

  @Override
  public List<Pose2d> getEstimatedPoses() {
    List<Pose2d> poses = new ArrayList<Pose2d>(m_latestEstimatedPoses.size());
    for (EstimatedRobotPose estimatedRobotPose : m_latestEstimatedPoses) {
      poses.add(estimatedRobotPose.estimatedPose.toPose2d());
    }
    return poses;
  }

  @Override
  public boolean hasTargetsInView() {
    for (CameraData cameraData : m_cameraData) {
      if (IVision.getLatestResultsWrapper(cameraData).hasTargets()) {
        return true;
      }
    }
    return false;
  }

  private Pose2d getEstimatedPose() {
    final Pose2d robotPose = m_latestEstimatedPoses.isEmpty()
        ? null
        : m_latestEstimatedPoses.get(0).estimatedPose.toPose2d();
    return robotPose;
  }

  // Note that this can provide multiple readings for a given target, if it can be
  // seen by more than 1 camera.
  //
  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on what we
    // can see.
    if (robotPose == null) {
      var estimatedPose = getEstimatedPose();
      if (estimatedPose == null) {
        // OK, can't estimate where we are, so bail out.
        return Collections.emptyList();
      }

      // Fine: now we think we know where we are.
      robotPose = estimatedPose;
    }

    List<TargetData> targets = new LinkedList<TargetData>();
    for (CameraData cameraData : m_cameraData) {
      targets.addAll(IVision.getTargetDataForCamera(cameraData, m_tagLayout, robotPose));
    }
    return targets;
  }

  @Override
  public List<TargetData> getVisibleTargets() {
    return getVisibleTargets(getEstimatedPose());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // SubsystemBase functions.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    super.periodic();

    // Where does the drive base think we are?
    final var optDrivePose = BulletinBoard.common.getValue(IDrivebasePlus.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Update camera-specific estimators (and gather their results).
    double lastTimestamp = 0;
    List<EstimatedRobotPose> estimates = new LinkedList<EstimatedRobotPose>();

    for (int i = 0; i < m_cameraData.size(); ++i) {
      final CameraData cameraData = m_cameraData.get(i);
      List<PhotonPipelineResult> results = cameraData.camera().getAllUnreadResults();
      if (results.isEmpty()) {
        // We don't have a new frame for this camera: we should (presumably) keep
        // the last results it supplied. (Which means we need to do some caching per
        // camera....)
        EstimatedRobotPose cachedEstimatedRobotPose = m_estimatedPosesCache.get(i);
        if (cachedEstimatedRobotPose != null) {
          estimates.add(cachedEstimatedRobotPose);
          lastTimestamp = Math.max(cachedEstimatedRobotPose.timestampSeconds, lastTimestamp);
        }
        continue;
      }
      // Camera processed a new frame since last
      // Get the last one in the list.
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        var estimate = updateEstimateForCamera(result, cameraData.estimator(), drivePose);
        if (!estimate.isEmpty()) {
          var estimatedPose = estimate.get();
          estimates.add(estimatedPose);
          lastTimestamp = Math.max(estimatedPose.timestampSeconds, lastTimestamp);

          // Update the cache
          m_estimatedPosesCache.put(i, estimatedPose);
        }
      }
    }

    // Save it, and publish it.
    m_latestEstimatedPoses = Collections.unmodifiableList(estimates);
    publishDataToBulletinBoard(!estimates.isEmpty(), lastTimestamp, m_latestEstimatedPoses);

    // System.out.println("Targets: " + getVisibleTargets());
  }
}
