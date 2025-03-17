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
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.CameraConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Vision processing implementation for a single/multiple cameras, using the
 * Photonvision libraries/server.
 */
public class Vision extends SubsystemBase implements IVision {
  /**
   * Camera data set.
   * 
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

  /** Entries for each of the cameras on the robot. */
  protected final List<CameraData> m_cameraData = new LinkedList<CameraData>();

  /** Estimated poses (if any) from the most recent camera data. */
  private List<EstimatedRobotPose> m_latestEstimatedPoses = Collections.emptyList();

  /**
   * Constructor.
   * 
   * @param config robot configuration, including camera data
   */
  public Vision(RobotConfig config) {
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

    // Add each of the cameras to our known set.
    List<RobotConfigs.CameraConfig> cameras = config.cameras();
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
    final Transform3d robotToCamera = new Transform3d(new Translation3d(cameraConfig.pos().x(), cameraConfig.pos().y(),
        cameraConfig.pos().z()),
        new Rotation3d(cameraConfig.orientation().roll(),
            cameraConfig.orientation().pitch(), cameraConfig.orientation().yaw()));
    final PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        m_tagLayout, POSE_STRATEGY, robotToCamera);
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

  /**
   * Publishes the latest pose data to the bulletin board from all cameras with
   * available data.
   * 
   * @param recentlyUpdated true if the data was updated recently
   * @param lastTimestamp   the timestamp of the last pose update
   * @param lastPoses       the last computed poses (if any)
   */
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

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // Functions exposed for simulation support.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Returns the list of CameraData records being used by this object.
   * 
   * Note: this is exposed as public only for use with SimVisionWrapper.
   * 
   * @return a list of CameraData objects
   */
  public final List<CameraData> getCameraDataForSimulation() {
    return getCameraData();
  }

  /**
   * The AprilTagFieldLayout being used by this object.
   * 
   * Note: this is exposed as public only for use with SimVisionWrapper.
   * 
   * @return an AprilTagFieldLayout
   */
  public final AprilTagFieldLayout getFieldLayoutForSimulation() {
    return m_tagLayout;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // IVision functions.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public List<EstimatedRobotPose> getEstimatedPoses() {
    return m_latestEstimatedPoses;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // SubsystemBase functions.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    super.periodic();

    // Where does the drive base think we are?
    final var optDrivePose = BulletinBoard.common.getValue(IDrivebase.ODOMETRY_KEY, Pose2d.class);
    final var drivePose = (Pose2d) (optDrivePose.isPresent() ? optDrivePose.get() : null);

    // Update camera-specific estimators (and gather their results).
    double lastTimestamp = 0;
    List<EstimatedRobotPose> estimates = new LinkedList<EstimatedRobotPose>();
    for (CameraData cameraData : m_cameraData) {
      var estimate = updateEstimateForCamera(
          cameraData.camera(), cameraData.estimator(),
          drivePose);
      if (!estimate.isEmpty()) {
        var estimatedPose = estimate.get();
        estimates.add(estimatedPose);
        lastTimestamp = Math.max(estimatedPose.timestampSeconds, lastTimestamp);
      }
    }

    // Save it, and publish it.
    m_latestEstimatedPoses = Collections.unmodifiableList(estimates);
    publishDataToBulletinBoard(!estimates.isEmpty(), lastTimestamp, m_latestEstimatedPoses);
  }
}
