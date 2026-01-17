// Copyright (c) 2024-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.util.RobotConfigs.CameraConfig;
import java.io.IOException;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Implements a PhotonVision-based (single-camera) vision subsystem.
 */
public class PhotonVision
    extends SubsystemBase implements IVisionPlus, IPhotonVision, IPoseEstimator {
  /** Data about a single camera used for vision tracking. */
  private final CameraData m_cameraData;

  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  private final AprilTagFieldLayout m_tagLayout;

  private Optional<EstimatedRobotPose> m_lastEstimatedPose = Optional.empty();

  /** Creates a new PhotonVision. */
  public PhotonVision(CameraConfig cameraConfig) {
    setName(SUBSYSTEM_NAME);
    m_tagLayout = IPhotonVision.loadLayout(FIELD_LAYOUT.m_resourceFile);

    // Add the first camera to our known set.
    // (Note that this assumes that we *have* at least one camera.)
    final PhotonCamera camera = new PhotonCamera(cameraConfig.name());
    final Transform3d robotToCamera = new Transform3d(
        new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(), cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(), cameraConfig.orientation().pitch(),
            cameraConfig.orientation().yaw()));
    var estimator = new PhotonPoseEstimator(m_tagLayout, robotToCamera);
    m_cameraData = new CameraData(camera, robotToCamera, estimator);
  }

  List<PhotonPipelineResult> m_pipelineResultsCache =
      Collections.unmodifiableList(Collections.emptyList());

  /**
   * Note: rthis should be invoked "exactly ONCE per loop of your robot code", per
   * PhotonVision docs. (Not frequently enough, and data is missed; too
   * frequently, and we'll drop frames within a loop.)
   */
  private void updateResultCache() {
    m_pipelineResultsCache =
        Collections.unmodifiableList(m_cameraData.camera().getAllUnreadResults());
  }

  /**
   * @return the latest vision pipeline results, based on m_pipelineResultsCache.
   */
  private PhotonPipelineResult getLatestResults() {
    if (m_pipelineResultsCache.isEmpty()) {
      return new PhotonPipelineResult();
    }

    final boolean SANITY_CHECK_ORDER = true;
    if (SANITY_CHECK_ORDER) {
      // Order *should* be well-defined, but it's unclear from docs if it's newest
      // first, or oldest.
      PhotonPipelineResult prior = null;
      for (var entry : m_pipelineResultsCache) {
        assert (prior == null || entry.getTimestampSeconds() <= prior.getTimestampSeconds());
        prior = entry;
      }
    }

    // Get the last one in the list, which *should* be the newest.
    PhotonPipelineResult result = m_pipelineResultsCache.get(m_pipelineResultsCache.size() - 1);
    return result;
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
  List<TargetData> getTargetData(AprilTagFieldLayout fieldLayout, Pose2d robotPose) {
    final var latestResults = getLatestResults();
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

  /**
   * Applies any updates for the estimator on a camera, optionally integrating the
   * last/reference pose provided by the drivebase.
   *
   * @param pipelineResult latest pipeline results from a camera
   * @param estimator      pose estimator being updated
   * @param referencePose  a reference pose (e.g., from the drive base); may be
   *                       null
   * @return the updated estimate, based on the camera data
   */
  protected static Optional<EstimatedRobotPose> updateEstimateForCamera(
      final PhotonPipelineResult pipelineResult, final PhotonPoseEstimator estimator,
      final Pose2d referencePose) {
    if (pipelineResult == null) {
      return Optional.empty();
    }

    // Note that per the PhotonVision docs
    // (https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization),
    // using estimateCoprocMultiTagPose is preferred. However, this requires
    // calibration of the camera(s), as well as enabling 3D-mode in the PhotonVision
    // UI.
    //
    // The docs also The recommended way to use the estimatePose methods is to do
    // estimation with one of MultiTag methods, check if the result is empty, then
    // fallback to single tag estimation using a method like
    // estimateLowestAmbiguityPose.
    //
    // If we have the drive pose (or some other reference, such as a prior fused
    // estimate) in which we have decent confidence, then we could also use methods
    // like estimateClosestToReferencePose as a fallback.
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
   * Updates pose estimation data posted to the BulletinBoard class, based on the
   * latest results.
   */
  private void updateBulletinBoard() {
    EstimatedPoseData estimate = null;
    if (m_lastEstimatedPose.isPresent()) {
      var lastPose = m_lastEstimatedPose.get();
      estimate =
          new EstimatedPoseData(lastPose.estimatedPose.toPose2d(), lastPose.timestampSeconds);
      BulletinBoard.common.updateValue(ESTIMATED_POSE_KEY, estimate);
      BulletinBoard.common.updateValue(ESTIMATED_POSE_SET_KEY, Collections.singletonList(estimate));
    } else {
      BulletinBoard.common.clearValue(ESTIMATED_POSE_KEY);
      BulletinBoard.common.clearValue(ESTIMATED_POSE_SET_KEY);
    }
  }

  /**
   * Updates m_lastEstimatedPose, based on current target data.
   */
  private void updateEstimatedPose() {
    // Where does the drive base think we are? (Note that if we're not going to use
    // this as a reference pose, then we don't need to get this data.)
    //
    // Note also that absent drive base data, we *could* also use a prior estimate
    // from vision, but having an *independent* reference can be useful.
    final var optDrivePose =
        BulletinBoard.common.getValue(IDrivebasePlus.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Build the estimate from the vision pipeline.
    List<PhotonPipelineResult> pipelineResultsList = m_pipelineResultsCache;
    if (!pipelineResultsList.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      PhotonPipelineResult result = pipelineResultsList.get(pipelineResultsList.size() - 1);
      m_lastEstimatedPose = updateEstimateForCamera(result, m_cameraData.estimator(), drivePose);
    } else {
      // We don't have a new frame for this camera: we should consider either caching
      // the last result frame we got, or the last estimated position (and allowing
      // whichever we cache to age out after some period).
      //
      // For now, we'll just hold onto the last estimate, since it includes a
      // timestamp that anything that wants to use it can check for "freshness".
    }
  }

  //
  // Methods from SubsystemBase
  //

  @Override
  public void periodic() {
    updateResultCache();
    updateEstimatedPose();
    updateBulletinBoard();
  }

  //
  // Methods from ISubsystem
  //

  @Override
  public void close() throws IOException {
    m_cameraData.camera().close();
  }

  //
  // Methods from IVision
  //

  @Override
  public boolean hasTargetsInView() {
    if (m_cameraData == null) {
      return false;
    }
    return getLatestResults().hasTargets();
  }

  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on
    // what we can see.
    if (robotPose == null) {
      // OK, can't estimate where we are, so bail out.
      return Collections.emptyList();
    }

    return getTargetData(m_tagLayout, robotPose);
  }

  //
  // Methods from IPhotonVision
  //

  @Override
  public List<CameraData> getCameraDataForSimulation() {
    return List.of(m_cameraData);
  }

  @Override
  public AprilTagFieldLayout getFieldLayoutForSimulation() {
    return m_tagLayout;
  }

  @Override
  public Optional<Pose2d> getEstimatedPose() {
    if (m_lastEstimatedPose.isEmpty()) {
      return Optional.empty();
    }

    // Notice that we're losing exposure to the "freshness timestamp" on the
    // estimate. This is something that we'd probably want to be careful about in
    // non-sample code....
    var lastPose = m_lastEstimatedPose.get();
    return Optional.of(lastPose.estimatedPose.toPose2d());
  }
}
