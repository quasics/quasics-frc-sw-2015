// Copyright (c) 2024-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

/**
 * Implements a PhotonVision-based vision subsystem, capable of handling an
 * arbitrary (non-zero) number of cameras.
 */
public class PhotonVisionAnyCameras extends SubsystemBase
    implements IVisionPlus, IPhotonVision, IPoseEstimator {
  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  private final AprilTagFieldLayout m_tagLayout;

  /** Data about the cameras used for vision tracking. */
  private final List<CameraData> m_cameraDataList;

  /**
   * Cached results from the vision pipeline for our cameras.
   */
  Map<CameraData, List<PhotonPipelineResult>> m_pipelineResultsCache = Collections
      .unmodifiableMap(Collections.emptyMap());

  /**
   * Cached estimated robot poses from each camera, based on the latest pipeline
   * results and pose estimation.
   */
  private Map<CameraData, EstimatedRobotPose> m_lastEstimatedPoses = new HashMap<>();

  /**
   * Constructor.
   * 
   * @param config robot configuration, including camera data
   */
  public PhotonVisionAnyCameras(RobotConfig config) {
    if (config.cameras().isEmpty()) {
      throw new IllegalArgumentException("At least one camera must be configured for PhotonVision");
    }

    setName(SUBSYSTEM_NAME);
    m_tagLayout = IPhotonVision.loadLayout(FIELD_LAYOUT.m_resourceFile);
    var cameraDataList = new LinkedList<CameraData>();

    for (CameraConfig cameraConfig : config.cameras()) {
      cameraDataList.add(allocateCameraData(cameraConfig));
    }
    m_cameraDataList = Collections.unmodifiableList(cameraDataList);
  }

  /**
   * Allocates a CameraData record for a given camera configuration, including
   * setting up the camera connection and pose estimator.
   * 
   * @param cameraConfig configuration for the camera, including name, position,
   *                     and orientation
   * @return a CameraData record with the initialized camera, transform, and
   *         estimator
   */
  private CameraData allocateCameraData(CameraConfig cameraConfig) {
    final PhotonCamera camera = new PhotonCamera(cameraConfig.name());
    final Transform3d robotToCamera = new Transform3d(new Translation3d(cameraConfig.pos().x(),
        cameraConfig.pos().y(), cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(),
            cameraConfig.orientation().pitch(),
            cameraConfig.orientation().yaw()));
    var estimator = new PhotonPoseEstimator(m_tagLayout, robotToCamera);
    return new CameraData(camera, robotToCamera, estimator);
  }

  /**
   * Updates cached results from the vision pipeline for our (single) camera.
   *
   * Note: this should be invoked "exactly ONCE per loop of your robot code",
   * per PhotonVision docs. (Not frequently enough, and data is missed; too
   * frequently, and we'll drop frames within a loop.)
   */
  private void updateResultCache() {
    m_pipelineResultsCache = new HashMap<>();
    for (CameraData cameraData : m_cameraDataList) {
      m_pipelineResultsCache.put(cameraData, cameraData.camera().getAllUnreadResults());
    }
  }

  /**
   * Utility method to get the latest vision pipeline results for a given camera.
   * 
   * @param cameraData the camera for which to get the latest results
   * @return the latest vision pipeline results, based on
   *         m_pipelineResultsCache.
   */
  private PhotonPipelineResult getLatestResults(CameraData cameraData) {
    if (m_pipelineResultsCache.isEmpty() || !m_pipelineResultsCache.containsKey(cameraData)) {
      return new PhotonPipelineResult();
    }

    var resultsForCamera = m_pipelineResultsCache.get(cameraData);
    if (resultsForCamera.isEmpty()) {
      return new PhotonPipelineResult();
    }

    final boolean SANITY_CHECK_ORDER = true;
    if (SANITY_CHECK_ORDER) {
      // Order *should* be well-defined, but it's unclear from docs if it's
      // newest first, or oldest.
      PhotonPipelineResult prior = null;
      for (var entry : resultsForCamera) {
        assert (prior == null
            || entry.getTimestampSeconds() <= prior.getTimestampSeconds());
        prior = entry;
      }
    }

    // Get the last one in the list, which *should* be the newest.
    PhotonPipelineResult result = resultsForCamera.get(resultsForCamera.size() - 1);
    return result;
  }

  /**
   * Returns estimated relative positioning data for all visible targets (if
   * any).
   * 
   * Note:
   * <ul>
   * <li>A target is "visible" if it's in the latest pipeline results for
   * any of the cameras.
   * <li>The relative positioning data is based on the robot's estimated position
   * and the known position of the target on the field.
   * <li>Targets may be visible in multiple cameras; if so, they will be included
   * multiple times in the returned list.
   * </ul>
   *
   * @param fieldLayout field layout, used to determine fixed (absolute)
   *                    positions for targets in view
   * @param robotPose   robot's estimated position (used to compute relative
   *                    positioning for targets)
   * @return estimated relative positioning data for all visible targets
   */
  List<TargetData> getTargetData(
      AprilTagFieldLayout fieldLayout,
      Pose2d robotPose) {
    List<TargetData> targets = new LinkedList<TargetData>();
    for (CameraData cameraData : m_cameraDataList) {
      final var latestResults = getLatestResults(cameraData);
      if (!latestResults.hasTargets()) {
        continue;
      }

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
    }

    return targets;
  }

  /**
   * Applies any updates for the estimator on a camera, optionally integrating
   * the last/reference pose provided by the drivebase.
   *
   * @param pipelineResult latest pipeline results from a camera
   * @param estimator      pose estimator being updated
   * @param referencePose  a reference pose (e.g., from the drive base); may be
   *                       null
   * @return the updated estimate, based on the camera data
   */
  protected static Optional<EstimatedRobotPose> updateEstimateForCamera(
      final PhotonPipelineResult pipelineResult,
      final PhotonPoseEstimator estimator,
      final Pose2d referencePose) {
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
   * Updates pose estimation data posted to the BulletinBoard class, based on
   * the latest results.
   */
  private void updateBulletinBoard() {
    if (m_lastEstimatedPoses.isEmpty()) {
      BulletinBoard.common.clearValue(ESTIMATED_POSE_KEY);
      BulletinBoard.common.clearValue(ESTIMATED_POSE_SET_KEY);
    } else {
      // Build a list of the latest estimates for each camera, and post it to the
      // bulletin board.
      List<EstimatedPoseData> lastPoseList = m_lastEstimatedPoses.values().stream()
          .map(lastPose -> new EstimatedPoseData(
              lastPose.estimatedPose.toPose2d(), lastPose.timestampSeconds))
          .toList();
      BulletinBoard.common.updateValue(
          ESTIMATED_POSE_SET_KEY, Collections.unmodifiableList(lastPoseList));

      // Also post the most recent estimate across all cameras as a separate entry.
      var mostRecentEntry = lastPoseList.stream()
          .max((entry1, entry2) -> Double.compare(entry1.timestamp(),
              entry2.timestamp()));
      BulletinBoard.common.updateValue(ESTIMATED_POSE_KEY, mostRecentEntry);
    }
  }

  /**
   * Updates m_pipelineResultsCache, based on current target data.
   * 
   * If we don't have a new frame for a given camera, we could consider either
   * caching the last result frame we got, or the last estimated position
   * (and allowing whichever we cache to age out after some period).
   *
   * For now, I'm just holding onto the last estimate (if any) for a given camera,
   * since it includes a timestamp that anything that wants to use it can check
   * for "freshness".
   */
  private void updateEstimatedPoses() {
    // Where does the drive base think we are? (Note that if we're not going to
    // use this as a reference pose, then we don't need to get this data.)
    //
    // Note also that absent drive base data, we *could* also use a prior
    // estimate from vision, but having an *independent* reference can be
    // useful.
    final var optDrivePose = BulletinBoard.common.getValue(
        IDrivebasePlus.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Walk through the pipeline results for each camera, and update the estimate
    // for each camera based on the latest results.
    m_pipelineResultsCache.forEach((cameraData, pipelineResultsList) -> {
      if (pipelineResultsList.isEmpty()) {
        // No new frames for this camera, so we won't update the estimate for this
        // camera.
        return;
      }

      PhotonPipelineResult result = pipelineResultsList.get(pipelineResultsList.size() - 1);
      var estimateOp = updateEstimateForCamera(result, cameraData.estimator(), drivePose);
      if (estimateOp.isPresent()) {
        var estimate = estimateOp.get();
        m_lastEstimatedPoses.put(cameraData, estimate); // Add/update the estimate for this camera.
      }
    });
  }

  //
  // Methods from SubsystemBase
  //

  @Override
  public void periodic() {
    updateResultCache();
    updateEstimatedPoses();
    updateBulletinBoard();
  }

  //
  // Methods from ISubsystem
  //

  @Override
  public void close() throws IOException {
    for (CameraData cameraData : m_cameraDataList) {
      cameraData.camera().close();
    }
  }

  //
  // Methods from IVision
  //

  @Override
  public boolean hasTargetsInView() {
    if (m_cameraDataList.isEmpty()) {
      return false;
    }

    for (CameraData cameraData : m_cameraDataList) {
      if (getLatestResults(cameraData).hasTargets()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    if (robotPose == null) {
      // OK, can't estimate where we are as a reference point, so bail out.
      return Collections.emptyList();
    }

    return getTargetData(m_tagLayout, robotPose);
  }

  //
  // Methods from IPhotonVision
  //

  @Override
  public List<CameraData> getCameraDataForSimulation() {
    return m_cameraDataList;
  }

  @Override
  public AprilTagFieldLayout getFieldLayoutForSimulation() {
    return m_tagLayout;
  }

  @Override
  public Optional<Pose2d> getEstimatedPose() {
    if (m_lastEstimatedPoses.isEmpty()) {
      return Optional.empty();
    }

    // Walk through the list and find the most recent estimate across all cameras.
    // Note that we could also consider doing some sort of fusion across the
    // estimates from different cameras, but for now we'll just take the most
    // recent one.
    var mostRecentEntry = m_lastEstimatedPoses.entrySet().stream()
        .max((entry1, entry2) -> Double.compare(entry1.getValue().timestampSeconds,
            entry2.getValue().timestampSeconds));
    if (mostRecentEntry.isEmpty()) {
      return Optional.empty();
    }

    // Notice that we're losing exposure to the "freshness timestamp" on the
    // estimate. This is something that we'd probably want to be careful about
    // in non-sample code....
    var lastPose = mostRecentEntry.get().getValue();
    return Optional.of(lastPose.estimatedPose.toPose2d());
  }
}
