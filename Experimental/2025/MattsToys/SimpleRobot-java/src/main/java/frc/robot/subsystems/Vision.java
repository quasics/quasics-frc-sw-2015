// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.utils.RobotConfigs.RobotConfig;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Vision processing implementation, based on Photonvision.
 */
public class Vision extends SubsystemBase implements IVision {
  /** Connection to our (single) camera. */
  protected final PhotonCamera m_camera;

  /** Defines the conversion from the robot's position, to ours. */
  protected final Transform3d m_robotToCamera;

  /**
   * Pose estimator from PhotonVision. Note that (per docs) the estimated poses
   * can have a lot of uncertainty/error baked into them when you are further away
   * from the targets.
   */
  private final PhotonPoseEstimator m_photonEstimator;

  /** Cached pose from last pose estimation update. */
  protected Optional<EstimatedRobotPose> m_lastEstimatedPose = Optional.empty();
  /** Timestamp of the last pose estimation update. */
  protected double m_lastEstTimestamp = 0;
  /** Has the pose estimate been updated recently? */
  protected boolean m_estimateRecentlyUpdated = false;

  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  protected final AprilTagFieldLayout m_tagLayout;

  private final PoseStrategy m_poseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
  private static final boolean USE_REEFSCAPE_LAYOUT = true;
  private static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;

  private static final AprilTagFields FIELD_LAYOUT = USE_REEFSCAPE_LAYOUT
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE ? AprilTagFields.k2025ReefscapeAndyMark
                                           : AprilTagFields.k2025ReefscapeWelded)
      : AprilTagFields.k2024Crescendo // Fall back on last year's game
      ;

  /**
   * Constructs a Vision subsystem, based on a specified robot configuration.
   *
   * @param config robot configuration
   */
  public Vision(RobotConfig config) {
    this(config.camera().name(),
        new Transform3d(new Translation3d(config.camera().pos().x(), config.camera().pos().y(),
                            config.camera().pos().z()),
            new Rotation3d(config.camera().orientation().roll(),
                config.camera().orientation().pitch(), config.camera().orientation().yaw())));
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
  private Vision(String cameraName, Transform3d robotToCameraTransform) {
    setName(SUBSYSTEM_NAME);

    // Set up the relative positioning of the camera.
    m_robotToCamera = robotToCameraTransform;

    // Connect to our camera. (May be a simulation.)
    if (cameraName != null && !cameraName.isBlank()) {
      m_camera = new PhotonCamera(cameraName);
    } else {
      // Need to set a value, since it's final, unless we're prepared to abort the
      // ctor. But we'll need to test for this elsewhere.
      m_camera = null;
    }

    // Load the layout of the AprilTags on the field.
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(FIELD_LAYOUT.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }
    m_tagLayout = tagLayout;

    //
    // Set up the vision pose estimator
    m_photonEstimator = new PhotonPoseEstimator(
        m_tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCamera);

    // Configure what to do in a multi-tag environment (like Crescendo) when only
    // one tag can be seen.
    m_photonEstimator.setMultiTagFallbackStrategy(m_poseStrategy);
  }

  /**
   * Updates/caches the latest estimated robot pose on the field from vision data,
   * which may be empty. This should only be called once per loop, and will be
   * invoked from our <code>periodic()</code> method.
   *
   * @see #periodic()
   */
  private void updateEstimatedGlobalPose() {
    if (m_camera == null) {
      // No camera? Nothing to do.
      return;
    }

    // Update the vision pose estimator with the latest robot pose from the drive
    // base.
    BulletinBoard.common.getValue(IDrivebase.POSE_KEY, Pose2d.class).ifPresentOrElse(pose -> {
      Pose2d pose2d = (Pose2d) pose;
      updateLastPose(pose2d);
      updateReferencePose(pose2d);
    }, () -> System.err.println("Warning: no robot drive pose available."));

    // Update the pose estimator with the latest vision measurements.
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    if (results.isEmpty()) {
      // No results? Nothing to do.
      return;
    }

    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    double lastEstimatedTimestamp = 0;
    for (PhotonPipelineResult photonPipelineResult : results) {
      lastEstimatedPose = m_photonEstimator.update(photonPipelineResult);
      lastEstimatedTimestamp = photonPipelineResult.getTimestampSeconds();
    }

    m_estimateRecentlyUpdated = Math.abs(lastEstimatedTimestamp - m_lastEstTimestamp)
        > VISION_TIMESTAMP_RECENCY_THRESHOLD_SECS;
    if (m_estimateRecentlyUpdated) {
      m_lastEstTimestamp = lastEstimatedTimestamp;
      m_lastEstimatedPose = lastEstimatedPose;
    }
  }

  // Note: this method will be called once per scheduler run
  @Override
  public void periodic() {
    super.periodic();

    updateEstimatedGlobalPose();
  }

  /**
   * Updates the pose that will be used as a basis for reference when the
   * CrossCheckWithReferencePose mode is selected.
   *
   * @param pose the reference pose to set
   */
  public void updateReferencePose(Pose2d pose) {
    if (m_photonEstimator != null) {
      m_photonEstimator.setReferencePose(pose);
    }
  }

  /**
   * Updates the pose that will be used as a basis for reference when the
   * AssumeMinimumMovement mode is selected.
   *
   * @param pose the reference pose to set
   */
  public void updateLastPose(Pose2d pose) {
    if (m_photonEstimator != null) {
      m_photonEstimator.setLastPose(pose);
    }
  }

  @Override
  public Optional<EstimatedRobotPose> getLastEstimatedPose() {
    return m_lastEstimatedPose;
  }

  @Override
  public double getLastEstTimestamp() {
    return m_lastEstTimestamp;
  }

  @Override
  public boolean getEstimateRecentlyUpdated() {
    return m_estimateRecentlyUpdated;
  }
}
