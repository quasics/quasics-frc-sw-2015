// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.BulletinBoard;

public class Vision extends SubsystemBase {
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

  public static final String VISION_POSE_KEY = "Vision.Pose";
  public static final String VISION_TIMESTAMP_KEY = "Vision.Timestamp";

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

  /** Creates a new Vision. */
  public Vision() {
    setName("Vision");

    // Set up the relative positioning of the camera.
    Translation3d robotToCameraTrl = new Translation3d(
        CAMERA_X.in(Meters),
        CAMERA_Y.in(Meters),
        CAMERA_HEIGHT.in(Meters));
    Rotation3d robotToCameraRot = new Rotation3d(
        CAMERA_ROLL.in(Radians),
        CAMERA_PITCH.in(Radians),
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
      tagLayout = AprilTagFieldLayout
          .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout.");
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
    BulletinBoard.common.getValue(IDrivebase.POSE_KEY, Pose2d.class).ifPresentOrElse(
        pose -> {
          m_photonEstimator.setReferencePose((Pose2d) pose);
          m_photonEstimator.setLastPose((Pose2d) pose);
        },
        () -> System.err.println("Warning: no robot drive pose available."));

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

    // // Compute the robot's field-relative position exclusively from vision
    // // measurements.
    // Pose3d visionMeasurement3d = objectToRobotPose(m_objectInField,
    // m_robotToCamera, m_cameraToObjectEntry);
    //
    // // Convert robot pose from Pose3d to Pose2d needed to apply vision
    // // measurements.
    // Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
    //
    // // Apply vision measurements. For simulation purposes only, we don't input a
    // // latency delay -- on a real robot, this must be calculated based either on
    // // known latency or timestamps.
    // m_poseEstimator.addVisionMeasurement(visionMeasurement2d,
    // Timer.getFPGATimestamp());

    m_estimateRecentlyUpdated = Math.abs(lastEstimatedTimestamp - m_lastEstTimestamp) > 1e-5;
    if (m_estimateRecentlyUpdated) {
      m_lastEstTimestamp = lastEstimatedTimestamp;
      m_lastEstimatedPose = lastEstimatedPose;
    }
  }

  // This method will be called once per scheduler run
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

  /**
   * Subclass to isolate simulation-specific code.
   */
  public static class SimulatedVision extends Vision {
    /**
     * Handles the nuts and bolts of the actual simulation, including wireframe
     * rendering.
     */
    protected VisionSystemSim m_visionSim = new VisionSystemSim("main");

    /** The interface to control/inject simulated camera stuff. */
    private PhotonCameraSim m_cameraSim = null;

    /** Constructor. */
    public SimulatedVision() {
      super();

      m_cameraSim = new PhotonCameraSim(m_camera, getCameraProperties());

      if (m_tagLayout != null) {
        m_visionSim.addAprilTags(m_tagLayout);
      } else {
        System.err.println("Warning: no April Tags layout loaded.");
      }

      // Add this camera to the vision system simulation with the given
      // robot-to-camera transform.
      m_visionSim.addCamera(m_cameraSim, m_robotToCamera);

      // Enable the raw and processed streams. (These are enabled by default, but I'm
      // making it explicit here, so that we can easily turn them off if we decide
      // that's needed.)
      //
      // These streams follow the port order mentioned in Camera Stream Ports. For
      // example, a single simulated camera will have its raw stream at localhost:1181
      // and processed stream at localhost:1182, which can also be found in the
      // CameraServer tab of Shuffleboard like a normal camera stream.
      m_cameraSim.enableRawStream(true);
      m_cameraSim.enableProcessedStream(true);

      // Enable drawing a wireframe visualization of the field to the camera streams.
      //
      // Note: This is extremely resource-intensive and is disabled by default.
      m_cameraSim.enableDrawWireframe(true);
    }

    /**
     * Returns the simulated camera properties (used to control properties like FOV,
     * resolution, etc.).
     * 
     * @see https://docs.photonvision.org/en/v2025.1.1/docs/simulation/simulation-java.html#camera-simulation
     */
    private SimCameraProperties getCameraProperties() {
      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(
          CAMERA_WIDTH_PX,
          CAMERA_HEIGHT_PX,
          Rotation2d.fromDegrees(CAMERA_FOV_DEG));
      cameraProp.setFPS(CAMERA_FPS);

      // Approximate detection noise with average and standard deviation error in
      // pixels.
      cameraProp.setCalibError(0.25, 0.08);

      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      return cameraProp;
    }

    // This method will be called once per scheduler run
    @Override
    public void simulationPeriodic() {
      // Should be a no-op, but good practice to call the base class.
      super.simulationPeriodic();

      Pose2d robotPoseMeters = (Pose2d) BulletinBoard.common.getValue(
          IDrivebase.POSE_KEY, Pose2d.class).orElse(new Pose2d());

      m_visionSim.update(robotPoseMeters);

      // Update the simulator to reflect where the estimated pose suggests that we
      // are located.
      final var debugField = m_visionSim.getDebugField();
      m_lastEstimatedPose.ifPresentOrElse(
          // Do this with the data in m_lastEstimatedPose (if it has some)
          est -> {
            debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d());
          },
          // If we have nothing in m_lastEstimatedPose, do this
          () -> {
            if (m_estimateRecentlyUpdated)
              debugField.getObject("VisionEstimation").setPoses();
          });
    }
  }
}
