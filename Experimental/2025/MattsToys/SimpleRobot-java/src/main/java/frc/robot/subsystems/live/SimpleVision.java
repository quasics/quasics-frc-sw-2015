// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class SimpleVision extends SubsystemBase implements IVision {
  final private CameraData m_cameraData;

  /** TODO: Move layout-oriented stuff into a common base for Simple/BetterVision. */

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

  /** Creates a new SimpleVision. */
  public SimpleVision(RobotConfig config) {
    setName(IVision.SUBSYSTEM_NAME);
    m_tagLayout = IVision.loadLayout(FIELD_LAYOUT.m_resourceFile);

    // Add the first camera to our known set.
    // (Note that this assumes that we *have* at least one camera.)
    final RobotConfigs.CameraConfig cameraConfig = config.cameras().get(0);
    final PhotonCamera camera = new PhotonCamera(cameraConfig.name());
    final Transform3d robotToCamera = new Transform3d(
        new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(), cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(), cameraConfig.orientation().pitch(),
            cameraConfig.orientation().yaw()));

    m_cameraData = new CameraData(camera, robotToCamera, null);
  }

  // Making this a helper function, since getLatestResult() is now deprecated, and
  // I'm trying to cut down on the number of warnings.
  private PhotonPipelineResult getLatestResult() {
    return m_cameraData.camera().getLatestResult();
  }

  @Override
  public boolean hasTargetsInView() {
    return getLatestResult().hasTargets();
  }

  /**
   * Helper method to try to estimate where the robot is on the field, if we can see any targets.
   *
   * @param fieldLayout  field layout (specifying what tags are where)
   * @param cameraData   camera information (providing visible targets and "robotToCamera" info)
   * @return estimate of where the robot is on the field, if we can see anything
   */
  private Optional<Pose3d> getEstimatedRobotPose(
      AprilTagFieldLayout fieldLayout, CameraData cameraData) {
    final var latestResults = getLatestResult();
    final var bestTarget = latestResults.getBestTarget();
    if (bestTarget == null) {
      // No targets in view
      return Optional.empty();
    }

    var bestTargetId = bestTarget.getFiducialId();
    if (!fieldLayout.getTagPose(bestTargetId).isPresent()) {
      // Target not in the field layout: huh?
      return Optional.empty();
    }

    // OK, do some math....
    var cameraToRobot = cameraData.transform3d().inverse();
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(),
        fieldLayout.getTagPose(bestTargetId).get(), cameraToRobot);
    return Optional.of(robotPose);
  }

  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on what we can see.
    if (robotPose == null) {
      var estimatedPose = getEstimatedRobotPose(m_tagLayout, m_cameraData).orElse(null);
      if (estimatedPose == null) {
        // OK, can't estimate where we are, so bail out.
        return Collections.emptyList();
      }

      // Fine: now we think we know where we are.
      robotPose = estimatedPose.toPose2d();
    }

    final var latestResults = getLatestResult();
    List<TargetData> targets = new LinkedList<TargetData>();
    for (var result : latestResults.targets) {
      var tagPose = m_tagLayout.getTagPose(result.fiducialId);
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
}
