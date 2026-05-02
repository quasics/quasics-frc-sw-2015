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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IPhotonVision;
import frc.robot.subsystems.interfaces.IPoseEstimator;
import frc.robot.subsystems.interfaces.IVisionPlus;
import frc.robot.util.BulletinBoard;
import frc.robot.util.config.CameraConfig;
import frc.robot.util.config.RobotConfig;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
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
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision processing implementation for a single/multiple cameras, using the
 * Photonvision libraries/server.
 */
public class PhotonVisionMultiCamera extends SubsystemBase
    implements IPhotonVision, IVisionPlus, IPoseEstimator {
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
  public PhotonVisionMultiCamera(RobotConfig config) {
    setName(IVisionPlus.SUBSYSTEM_NAME);
    m_tagLayout = IPhotonVision.loadLayout(FIELD_LAYOUT.m_resourceFile);

    // Add each of the cameras to our known set.
    List<CameraConfig> cameras = config.cameras();
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
    final PhotonPoseEstimator estimator = new PhotonPoseEstimator(m_tagLayout, robotToCamera);
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

    // Note that per the PhotonVision docs
    // (https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization),
    // using estimateCoprocMultiTagPose is preferred. However, this requires
    // calibration of the camera(s), as well as enabling 3D-mode in the
    // PhotonVision UI.
    //
    // The docs also The recommended way to use the estimatePose methods is to
    // do estimation with one of MultiTag methods, check if the result is empty,
    // then fallback to single tag estimation using a method like
    // estimateLowestAmbiguityPose.
    //
    // If we have the drive pose (or some other reference, such as a prior fused
    // estimate) in which we have decent confidence, then we could also use
    // methods like estimateClosestToReferencePose as a fallback.
    var result = estimator.estimateCoprocMultiTagPose(
        pipelineResult); // Or
                         // estimator.estimateAverageBestTargetsPose(pipelineResult),
                         // etc.
    if (result.isEmpty()) {
      result = estimator.estimateLowestAmbiguityPose(pipelineResult);
    }
    return result;
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
      BulletinBoard.common.clearValue(ESTIMATED_POSE_KEY);
      BulletinBoard.common.clearValue(ESTIMATED_POSE_SET_KEY);
    } else {
      var lastPose = lastPoses.get(0);
      EstimatedPoseData estimate = new EstimatedPoseData(
          lastPose.estimatedPose.toPose2d(), lastPose.timestampSeconds);
      BulletinBoard.common.updateValue(ESTIMATED_POSE_KEY, estimate);
      BulletinBoard.common.updateValue(ESTIMATED_POSE_SET_KEY, lastPoses);
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
  public Optional<Pose2d> getEstimatedPose() {
    // TODO: this is pretty rough, and just returns the first one in the list. We
    // should probably be doing something smarter here, especially if we have
    // multiple cameras with different views of the field.
    if (m_latestEstimatedPoses.isEmpty()) {
      return Optional.empty();
    }

    // Notice that we're losing exposure to the "freshness timestamp" on the
    // estimate. This is something that we'd probably want to be careful about
    // in non-sample code....
    var lastPose = m_latestEstimatedPoses.get(0);
    return Optional.of(lastPose.estimatedPose.toPose2d());
  }

  public List<Pose2d> getEstimatedPoses() {
    List<Pose2d> poses = new ArrayList<Pose2d>(m_latestEstimatedPoses.size());
    for (EstimatedRobotPose estimatedRobotPose : m_latestEstimatedPoses) {
      poses.add(estimatedRobotPose.estimatedPose.toPose2d());
    }
    return poses;
  }

  /**
   * Cached results from the vision pipeline for our (single) camera.
   */
  Map<CameraData, List<PhotonPipelineResult>> m_pipelineResultsCache = new HashMap<CameraData, List<PhotonPipelineResult>>();

  /**
   * Updates cached results from the vision pipeline for our (single) camera.
   *
   * Note: rthis should be invoked "exactly ONCE per loop of your robot code",
   * per PhotonVision docs. (Not frequently enough, and data is missed; too
   * frequently, and we'll drop frames within a loop.)
   */
  private void updateResultCache() {
    m_pipelineResultsCache = new HashMap<CameraData, List<PhotonPipelineResult>>();
    for (CameraData cameraData : m_cameraData) {
      m_pipelineResultsCache.put(cameraData, cameraData.camera().getAllUnreadResults());
    }
  }

  /**
   * @return the latest vision pipeline results, based on
   *         m_pipelineResultsCache.
   */
  private PhotonPipelineResult getLatestResults(CameraData cameraData) {
    var cachedResults = m_pipelineResultsCache.get(cameraData);
    if (cachedResults == null || cachedResults.isEmpty()) {
      return new PhotonPipelineResult();
    }

    final boolean SANITY_CHECK_ORDER = true;
    if (SANITY_CHECK_ORDER) {
      // Order *should* be well-defined, but it's unclear from docs if it's
      // newest first, or oldest.
      PhotonPipelineResult prior = null;
      for (var entry : cachedResults) {
        assert (prior == null
            || entry.getTimestampSeconds() <= prior.getTimestampSeconds());
        prior = entry;
      }
    }

    // Get the last one in the list, which *should* be the newest.
    PhotonPipelineResult result = cachedResults.get(cachedResults.size() - 1);
    return result;
  }

  @Override
  public boolean hasTargetsInView() {
    for (CameraData cameraData : m_cameraData) {
      var latestResults = getLatestResults(cameraData);
      if (latestResults.hasTargets()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns estimated relative positioning data for all visible targets (if
   * any).
   *
   * @param fieldLayout field layout, used to determine fixed (absolute)
   *                    positions
   *                    for targets in
   *                    view
   * @param robotPose   robot's estimated position (used to compute relative
   *                    positioning for targets)
   * @return estimated relative positioning data for all visible targets
   */
  List<TargetData> getTargetData(
      AprilTagFieldLayout fieldLayout, Pose2d robotPose, PhotonPipelineResult pipelineResult) {
    if (!pipelineResult.hasTargets()) {
      return Collections.emptyList();
    }

    List<TargetData> targets = new LinkedList<TargetData>();
    for (PhotonTrackedTarget result : pipelineResult.targets) {
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

  // Note that this can provide multiple readings for a given target, if it can be
  // seen by more than 1 camera.
  //
  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on what we
    // can see.
    if (robotPose == null) {
      var estimatedPose = getEstimatedPose();
      if (estimatedPose.isEmpty()) {
        // OK, can't estimate where we are, so bail out.
        return Collections.emptyList();
      }

      // Fine: now we think we know where we are.
      robotPose = estimatedPose.get();
    }

    // Walk the data for each camera, and gather up all the targets we can see.
    List<TargetData> targets = new LinkedList<TargetData>();
    for (CameraData cameraData : m_cameraData) {
      var latestResults = getLatestResults(cameraData);
      if (latestResults == null || !latestResults.hasTargets()) {
        // No targets in view for this camera, so skip it.
        continue;
      }
      targets.addAll(getTargetData(m_tagLayout, robotPose, latestResults));
    }
    return targets;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // SubsystemBase functions.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    super.periodic();
    updateResultCache();

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

  @Override
  public void close() throws IOException {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
}
