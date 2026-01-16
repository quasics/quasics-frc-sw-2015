// Copyright (c) FIRST and other WPILib contributors.
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
import frc.robot.subsystems.interfaces.IPhotonVision;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.util.RobotConfigs.CameraConfig;

import java.io.IOException;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Implements a PhotonVision-based (single-camera) vision subsystem.
 */
public class PhotonVision extends SubsystemBase implements IVision, IPhotonVision {
  /** Data about a single camera used for vision tracking. */
  private final CameraData m_cameraData;

  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  private final AprilTagFieldLayout m_tagLayout;

  /** Creates a new PhotonVision. */
  public PhotonVision(CameraConfig cameraConfig) {
    setName(SUBSYSTEM_NAME);
    m_tagLayout = IPhotonVision.loadLayout(FIELD_LAYOUT.m_resourceFile);

    // Add the first camera to our known set.
    // (Note that this assumes that we *have* at least one camera.)
    final PhotonCamera camera = new PhotonCamera(cameraConfig.name());
    final Transform3d robotToCamera = new Transform3d(
        new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(),
            cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(),
            cameraConfig.orientation().pitch(),
            cameraConfig.orientation().yaw()));

    m_cameraData = new CameraData(camera, robotToCamera, null);
  }

  // Making this a helper function, since getLatestResult() is now deprecated,
  // and I'm trying to cut down on the number of warnings.
  private static PhotonPipelineResult getLatestResultsWrapper(CameraData cameraData) {
    // TODO: look at replacing this with something in a reusable base class to
    // try to cache data, to handle the deprecation.
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
  static List<TargetData> getTargetDataForCamera(CameraData cameraData, AprilTagFieldLayout fieldLayout,
      Pose2d robotPose) {
    final var latestResults = getLatestResultsWrapper(cameraData);
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

  //
  // Methods from SubsystemBase
  //

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //
  // Methods from IVision
  //

  @Override
  public boolean hasTargetsInView() {
    if (m_cameraData == null) {
      return false;
    }
    return getLatestResultsWrapper(m_cameraData).hasTargets();
  }

  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on
    // what we can see.
    if (robotPose == null) {
      // OK, can't estimate where we are, so bail out.
      return Collections.emptyList();
    }

    return getTargetDataForCamera(m_cameraData, m_tagLayout, robotPose);
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
  public void close() throws IOException {
    m_cameraData.camera().close();
  }
}
