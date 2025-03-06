// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.abstracts;

import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.utils.BulletinBoard;

/**
 * Abstract base class for vision-handling, unifying some single/multi-camera
 * stuff.
 */
public abstract class AbstractVision extends SubsystemBase implements IVision {
  /**
   * @param camera      connection to the camera
   * @param transform3d defines the conversion from the robot's position, to the
   *                    cameras's
   * @param estimator   pose estimator associated with this camera. Note that (per
   *                    docs) the estimated poses can have a lot of
   *                    uncertainty/error baked into them when you are further
   *                    away from the targets.
   */
  public record CameraData(PhotonCamera camera, Transform3d transform3d, PhotonPoseEstimator estimator) {
  }

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

  /** Creates a new AbstractVision. */
  public AbstractVision() {
    setName(IVision.SUBSYSTEM_NAME);

    // Load the layout of the AprilTags on the field.
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(FIELD_LAYOUT.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }
    m_tagLayout = tagLayout;
  }

  /**
   * Applies any updates for the estimator on a camera, optionally integrating the
   * last/reference pose provided by the drivebase.
   * 
   * @param camera    camera supplying data to use in the estimate
   * @param estimator pose estimator being updated
   * @param drivePose last reported pose from the drivebase (or null)
   * @return the updated estimate, based on the camera data
   */
  protected static Optional<EstimatedRobotPose> updateEstimateForCamera(
      final PhotonCamera camera,
      final PhotonPoseEstimator estimator,
      final Pose2d drivePose) {
    // Update the vision pose estimator with the latest robot pose from the drive
    // base (if we have one).
    if (drivePose != null) {
      estimator.setLastPose(drivePose);
      estimator.setReferencePose(drivePose);
    }

    // Update the pose estimator with the latest vision measurements from its
    // camera.
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      // No results? Nothing to do.
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    for (PhotonPipelineResult photonPipelineResult : results) {
      lastEstimatedPose = estimator.update(photonPipelineResult);
    }

    return lastEstimatedPose;
  }

  protected static void publishDataToBulletinBoard(
      boolean recentlyUpdated,
      double lastTimestamp,
      Optional<EstimatedRobotPose> lastPose) {
    publishDataToBulletinBoard(recentlyUpdated, lastTimestamp,
        lastPose.isPresent() ? Collections.singletonList(lastPose.get()) : null);
  }

  protected static void publishDataToBulletinBoard(
      boolean recentlyUpdated,
      double lastTimestamp,
      List<EstimatedRobotPose> lastPoses) {
    if (!recentlyUpdated || lastPoses == null || lastPoses.isEmpty()) {
      BulletinBoard.common.clearValue(POSE_TIMESTAMP_KEY);
      BulletinBoard.common.clearValue(POSES_KEY);
    } else {
      BulletinBoard.common.updateValue(POSE_TIMESTAMP_KEY, lastTimestamp);
      BulletinBoard.common.updateValue(POSES_KEY, lastPoses);
    }
  }

  /** @return the number of cameras being used */
  public abstract int getNumCameras();

  /** @return the list of CameraData objects for this subsystem */
  protected abstract List<CameraData> getCameraData();
}
