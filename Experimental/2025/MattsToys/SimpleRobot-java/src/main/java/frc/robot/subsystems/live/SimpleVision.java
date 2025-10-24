// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.vision.IVision;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * A simple class for handling a single camera and determining estimated
 * distance/angle for any
 * visible targets.
 */
public class SimpleVision extends SubsystemBase implements IVision {
  final private CameraData m_cameraData;

  /**
   * TODO: Move layout-oriented stuff into a common base for Simple/BetterVision.
   */

  private static final boolean USE_REEFSCAPE_LAYOUT = true;
  private static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;
  private static final AprilTagFields FIELD_LAYOUT = USE_REEFSCAPE_LAYOUT
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE ? AprilTagFields.k2025ReefscapeAndyMark
          : AprilTagFields.k2025ReefscapeWelded)
      : AprilTagFields.k2024Crescendo // Fall back on last year's game
  ;
  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  protected final AprilTagFieldLayout m_tagLayout;

  /**
   * Creates a new SimpleVision.
   *
   * @param config configuration of the underlying robot (which *must* have at
   *               least 1 camera)
   */
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

  @Override
  public AprilTagFieldLayout getFieldLayoutForSimulation() {
    return m_tagLayout;
  }

  @Override
  public List<CameraData> getCameraDataForSimulation() {
    return Collections.singletonList(m_cameraData);
  }

  @Override
  public List<Pose2d> getEstimatedPoses() {
    var estimate = getEstimatedRobotPose();
    if (estimate.isPresent()) {
      return Collections.singletonList(estimate.get().toPose2d());
    } else {
      return Collections.emptyList();
    }
  }

  @Override
  public boolean hasTargetsInView() {
    return IVision.getLatestResultsWrapper(m_cameraData).hasTargets();
  }

  /**
   * Helper method to try to estimate where the robot is on the field, using the
   * best (single)
   * target that we can see, if any.
   *
   * @param fieldLayout field layout (specifying what tags are where)
   * @param cameraData  camera information (providing visible targets and
   *                    "robotToCamera" info)
   * @return estimate of where the robot is on the field, if we can see anything
   */
  private Optional<Pose3d> getEstimatedRobotPose() {
    final var latestResults = IVision.getLatestResultsWrapper(m_cameraData);
    if (latestResults.hasTargets()) {
      // No targets in view
      return Optional.empty();
    }

    final var bestTarget = latestResults.getBestTarget();
    var bestTargetId = bestTarget.getFiducialId();
    if (!m_tagLayout.getTagPose(bestTargetId).isPresent()) {
      // Target not in the field layout: huh?
      return Optional.empty();
    }

    // OK, do some math....
    var cameraToRobot = m_cameraData.transform3d().inverse();
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(),
        m_tagLayout.getTagPose(bestTargetId).get(), cameraToRobot);
    return Optional.of(robotPose);
  }

  // TODO: Test this....
  @Override
  public List<TargetData> getVisibleTargets(Pose2d robotPose) {
    // If the caller didn't give us a pose, then try to estimate it based on what we
    // can see.
    if (robotPose == null) {
      var estimatedPose = getEstimatedRobotPose();
      if (estimatedPose.isEmpty()) {
        // OK, can't estimate where we are, so bail out.
        return Collections.emptyList();
      }

      // Fine: now we think we know where we are.
      robotPose = estimatedPose.get().toPose2d();
    }

    return IVision.getTargetDataForCamera(m_cameraData, m_tagLayout, robotPose);
  }
}
