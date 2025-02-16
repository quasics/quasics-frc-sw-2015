// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.utils.BulletinBoard;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase implements IVision {
  // The camera properties, relative to the center of the robot (and ground
  // level).
  //
  // TODO: Update these values to reflect the real camera's properties.
  //
  // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
  // pose (which is considered to be its center of rotation at the floor level, or
  // Z = 0)...
  public static final Distance CAMERA_HEIGHT = Meters.of(0.5); // meters
  public static final Distance CAMERA_X = Meters.of(0.1); // meters
  public static final Distance CAMERA_Y = Meters.of(0.0); // meters
  // ...pitched 15 degrees up, pointing straightforward and in plane with the
  // robot,...
  public static final Angle CAMERA_PITCH = Degrees.of(-15); // pointed 15 degrees up
  public static final Angle CAMERA_ROLL = Degrees.of(0); // degrees
  public static final Angle CAMERA_YAW = Degrees.of(0); // degrees
  // ...with image dimensions, field of view, FPS being this...
  public static final int CAMERA_WIDTH_PX = 960; // pixels
  public static final int CAMERA_HEIGHT_PX = 720; // pixels
  public static final int CAMERA_FOV_DEG = 100; // degrees
  public static final int CAMERA_FPS = 20; // frames per second (limited by robot loop rate)
  // ...and is named as follows.
  public static final String CAMERA_NAME = "cameraName";

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

  // Cached results of the last pose estimation update.
  protected Optional<EstimatedRobotPose> m_lastEstimatedPose = Optional.empty();
  protected double m_lastEstTimestamp = 0;
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
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE
          ? AprilTagFields.k2025ReefscapeAndyMark
          : AprilTagFields.k2025ReefscapeWelded)
      : AprilTagFields.k2024Crescendo // Fall back on last year's game
  ;

  /** Creates a new Vision. */
  public Vision() {
    setName(SUBSYSTEM_NAME);

    // Set up the relative positioning of the camera.
    Translation3d robotToCameraTrl = new Translation3d(CAMERA_X.in(Meters), CAMERA_Y.in(Meters),
        CAMERA_HEIGHT.in(Meters));
    Rotation3d robotToCameraRot = new Rotation3d(CAMERA_ROLL.in(Radians), CAMERA_PITCH.in(Radians),
        CAMERA_YAW.in(Radians));
    m_robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    // Connect to our camera. (May be a simulation.)
    if (CAMERA_NAME != null && !CAMERA_NAME.isBlank()) {
      m_camera = new PhotonCamera(CAMERA_NAME);
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

    m_estimateRecentlyUpdated = Math
        .abs(lastEstimatedTimestamp - m_lastEstTimestamp) > VISION_TIMESTAMP_RECENCY_THRESHOLD_SECS;
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
