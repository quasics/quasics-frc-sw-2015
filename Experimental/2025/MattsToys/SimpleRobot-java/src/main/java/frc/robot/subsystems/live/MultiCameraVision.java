// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import java.io.IOException;
import java.util.Collections;
import java.util.LinkedList;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs.CameraConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Vision processing implementation for a single/multiple cameras, using the
 * Photonvision libraries/server.
 */
public class MultiCameraVision extends SubsystemBase implements IVision {
  /**
   * @param camera      connection to the camera
   * @param transform3d defines the conversion from the robot's position, to the
   *                    cameras's
   * @param estimator   pose estimator associated with this camera
   */
  protected record CameraData(PhotonCamera camera, Transform3d transform3d, PhotonPoseEstimator estimator) {
  }

  /** Entries for each of the cameras on the robot. */
  protected final List<CameraData> m_cameraData = new LinkedList<CameraData>();

  private List<EstimatedRobotPose> m_latestEstimatedPoses = Collections.emptyList();

  private static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

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

  /**
   * Constructor.
   * 
   * @param config robot configuration, including camera data
   */
  public MultiCameraVision(RobotConfig config) {
    // Load the layout of the AprilTags on the field.
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(FIELD_LAYOUT.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }
    m_tagLayout = tagLayout;

    // Add each of the cameras to our known set.
    // TODO: Update this to deal with multi-cam, once the robot config supports
    // that.
    addCameraToSet(config.camera());
  }

  /**
   * Adds a camera to the known set.
   * 
   * Note: must be invoked *after* the tag layout has been loaded.
   * 
   * @param cameraConfig
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

  private static Optional<EstimatedRobotPose> updateEstimateForCamera(
      CameraData cameraData,
      Pose2d drivePose) {
    final var estimator = cameraData.estimator();

    // Update the vision pose estimator with the latest robot pose from the drive
    // base (if we have one).
    if (drivePose != null) {
      estimator.setLastPose(drivePose);
      estimator.setReferencePose(drivePose);
    }

    // Update the pose estimator with the latest vision measurements from its
    // camera.
    List<PhotonPipelineResult> results = cameraData.camera().getAllUnreadResults();
    if (results.isEmpty()) {
      // No results? Nothing to do.
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    for (PhotonPipelineResult photonPipelineResult : results) {
      lastEstimatedPose = cameraData.estimator().update(photonPipelineResult);
    }

    return lastEstimatedPose;
  }

  @Override
  public void periodic() {
    super.periodic();

    // Where does the drive base think we are?
    final var optDrivePose = BulletinBoard.common.getValue(IDrivebase.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Update camera-specific estimators (and gather their results).
    List<EstimatedRobotPose> estimates = new LinkedList<EstimatedRobotPose>();
    for (CameraData cameraData : m_cameraData) {
      var estimate = updateEstimateForCamera(cameraData, drivePose);
      if (!estimate.isEmpty()) {
        estimates.add(estimate.get());
      }
    }

    // Save it, and publish it.
    m_latestEstimatedPoses = Collections.unmodifiableList(estimates);
    if (!m_latestEstimatedPoses.isEmpty()) {
      BulletinBoard.common.updateValue(POSES_KEY, m_latestEstimatedPoses);
    } else {
      BulletinBoard.common.clearValue(POSES_KEY);
    }
  }

  @Override
  public List<EstimatedRobotPose> getEstimatedPoses() {
    return m_latestEstimatedPoses;
  }
}
