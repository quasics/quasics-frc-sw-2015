// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SimulationSupport;
import java.util.Optional;
import java.util.function.BiFunction;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A simple vision-processing subsystem, based on PhotonVision.
 *
 * This includes simulator support, as well as some initial code to enable the
 * integration of vision-based pose estimation (using AprilTags for pose
 * estimates) into the robot's planning.
 * 
 * When running under simulation, the raw camera stream (showing the robot's
 * simulated "forward view") will be available at http://localhost:1181, and the
 * processed stream is at http://localhost:1182. They can also be found in the
 * CameraServer tab of Shuffleboard, like a normal camera stream.
 */
public class VisionSubsystem extends SubsystemBase {
  /**
   * A private enum (for local naming/documentation in this example), to use in
   * controlling how pose estimation should be handled when only one target can be
   * seen. In such cases, there may be a couple of possible poses that can be
   * calculated, due to ambiguity in the vision data, so the options are largely
   * focused on how to come up with a single answer in this case.
   * 
   * Note that in "production code", I likely wouldn't bother defining one enum
   * that simply maps to another; I'd just use the enum from PhotonVision. (But
   * providing a venue for additional documentation is helpful in sample code.)
   */
  public enum EstimationMode {
    /**
     * Given multiple possible poses, use the one that's closest to the
     * last-calculated pose.
     */
    AssumeMinimumMovement(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE),
    /** Take a weighted average of all possible poses. */
    UseAverageOfEstimatedPoses(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS),
    /** Choose the pose with the lowest ambiguity ("single best guess"). */
    MinimalAmbiguity(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY),
    /**
     * Take the estimate that most closely matches the known height of the camera.
     * (Obviously, this won't necessarily work if we're climbing, etc.)
     */
    CrossCheckWithCameraHeight(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT),
    /**
     * Cross-check with the last-provided reference pose (e.g., something taken
     * *back* from the drive base). (Obviously, this will only work well if we're
     * feeding information back-and-forth between the estimation in the drive base
     * and the estimates from the camera data.)
     */
    CrossCheckWithReferencePose(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    EstimationMode(PhotonPoseEstimator.PoseStrategy strategy) {
      m_strategy = strategy;
    }

    final PhotonPoseEstimator.PoseStrategy m_strategy;
  }

  /**
   * The default mode for what to do in a multi-tag environment (like Crescendo)
   * when only one tag can be seen.
   */
  private static EstimationMode DEFAULT_ESTIMATION_MODE = EstimationMode.AssumeMinimumMovement;

  /**
   * Link to the camera data stream from PhotonVision, used to pull in tracking
   * data.
   */
  private final PhotonCamera m_camera;

  /**
   * The layout of the AprilTags on the field. This is used for the pose
   * estimation (as well as in the simulator, when it's rendering the tag).
   */
  public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  /**
   * Pose estimator from PhotonVision. Note that (per docs) the estimated poses
   * can have a lot of uncertainty/error baked into them when you are further away
   * from the targets.
   */
  private final PhotonPoseEstimator m_photonEstimator;

  private final EstimationMode m_estimationMode;

  /** Camera mounting information, relative to the robot's centerpoint. */
  private final Transform3d kRobotToCam;

  /** Constructor. */
  public VisionSubsystem(RobotSettings.Robot robot) {
    this(robot, DEFAULT_ESTIMATION_MODE);
  }

  /** Constructor. */
  public VisionSubsystem(RobotSettings.Robot robot, EstimationMode mode) {
    this(robot.cameraName, robot.robotToCameraTransform, mode);
  }

  /**
   * Root constructor, to which all of the others delegate the real work.
   * 
   * @param cameraName    name under which the camera is publishing its data
   * @param robotToCamera camera mounting transformation (used to convert
   *                      robot-centered poses to camera-centered)
   * @param mode          how we should handle pose estimation (when only 1 target
   *                      is seen)
   */
  private VisionSubsystem(String cameraName, Transform3d robotToCamera, EstimationMode mode) {
    if (cameraName == null || cameraName.isEmpty()) {
      // OK: this robot has no camera on it, so setup is basically trivial.
      m_camera = null;
      kRobotToCam = null;
      m_photonEstimator = null;
      m_estimationMode = mode;
      return;
    }

    //
    // Core setup
    m_camera = new PhotonCamera(cameraName);
    kRobotToCam = robotToCamera;
    m_estimationMode = mode;

    //
    // Set up the vision pose estimator
    m_photonEstimator = new PhotonPoseEstimator(
        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera,
        kRobotToCam);

    // Configure what to do in a multi-tag environment (like Crescendo) when only
    // one tag can be seen.
    m_photonEstimator.setMultiTagFallbackStrategy(m_estimationMode.m_strategy);

    //
    // ----- Simulation support
    if (Robot.isSimulation()) {
      setupSimulationSupport();
    }
  }

  public EstimationMode getEstimationMode() {
    return m_estimationMode;
  }

  /** @return the latest result data from the camera. */
  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  public Optional<PhotonTrackedTarget> getMatchedTarget(int tagId) {
    final var latestResult = getLatestResult();
    if (!latestResult.hasTargets()) {
      return Optional.empty();
    }

    return latestResult.getTargets()
        .stream() // Generates a (lazy-evaluated) stream of tracked targets to be considered
        .filter(t -> t.getFiducialId() == tagId) // Filters the stream to those matching our target ID
        .findFirst() // Limits evaluation to just the 1st match
    ;
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Pose estimation support
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** A trivial (null-op) consumer of the pose data. */
  final public BiFunction<Pose2d, Double, Void> NULL_ESTIMATOR_FUNCTION = (P, D) -> {
    // poseEstimator.addVisionMeasurement(P, D);
    return null;
  };

  /**
   * The function to be invoked whenever we have new pose data from the
   * estimator.
   */
  BiFunction<Pose2d, Double, Void> m_poseEstimatorFunction = NULL_ESTIMATOR_FUNCTION;

  /**
   * Sets a function that we should invoke whenever we have new pose data from
   * our estimator.
   */
  public void setPoseEstimatorConsumer(BiFunction<Pose2d, Double, Void> consumer) {
    m_poseEstimatorFunction = consumer != null ? consumer : NULL_ESTIMATOR_FUNCTION;
  }

  private Optional<EstimatedRobotPose> m_lastEstimatedPose = Optional.empty();
  private double m_lastEstTimestamp = 0;
  private boolean m_estimateRecentlyUpdated = false;

  /**
   * Returns the latest estimated robot pose on the field from vision data. This
   * may be empty, and will only be updated once per loop.
   */
  public Optional<EstimatedRobotPose> getLastEstimatedPose() {
    return m_lastEstimatedPose;
  }

  /**
   * The latest estimated robot pose on the field from vision data, which may be
   * empty. This should only be called once per loop, and will be invoked from
   * our <code>periodic()</code> method.
   *
   * @see #periodic()
   */
  private void updateEstimatedGlobalPose() {
    if (m_camera == null) {
      // No camera? Nothing to do.
      return;
    }

    m_lastEstimatedPose = m_photonEstimator.update();

    final double latestTimestamp = m_camera.getLatestResult().getTimestampSeconds();
    m_estimateRecentlyUpdated = Math.abs(latestTimestamp - m_lastEstTimestamp) > 1e-5;

    if (m_estimateRecentlyUpdated) {
      m_lastEstTimestamp = latestTimestamp;
    }
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Overridden Subsystem behavior
  //
  /////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    updateEstimatedGlobalPose();

    // Update anyone else (e.g., the drive base) that would like to know where
    // the vision data suggests that the robot might be located.
    if (m_lastEstimatedPose.isPresent()) {
      var estimate = m_lastEstimatedPose.get();
      m_poseEstimatorFunction.apply(estimate.estimatedPose.toPose2d(),
          estimate.timestampSeconds);
    }
  }

  @Override
  public void simulationPeriodic() {
    var possiblePose = SimulationSupport.getSimulatedPose();
    if (possiblePose.isPresent()) {
      // Update the simulator data to reflect where the robot thinks it's
      // located.
      visionSim.update(possiblePose.get());
    }

    // Update the simulator to reflect where the estimated pose suggests that we
    // are located.
    if (Robot.isSimulation()) {
      m_lastEstimatedPose.ifPresentOrElse(
          // Do this with the data in m_lastEstimatedPose (if it has some)
          est -> getSimDebugField()
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          // If we have nothing in m_lastEstimatedPose, do this
          () -> {
            if (m_estimateRecentlyUpdated)
              getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Custom simulation support
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * If true, include wireframe rendering on raw video during simulation. (Note
   * that this will slow things down.)
   */
  final boolean ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO = true;

  // Simulation data
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  private void setupSimulationSupport() {
    if (Robot.isReal()) {
      return;
    }
    // Create the vision system simulation which handles cameras and targets
    // on the field.
    visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(kTagLayout);

    // Create simulated camera properties. These can be set to mimic your
    // actual camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's
    // values with visible targets.
    cameraSim = new PhotonCameraSim(m_camera, cameraProp);

    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, kRobotToCam);

    // Draw a wireframe of the visual field on the raw video stream. (This
    // will significantly increase loop times, so this should be false if
    // we're not using the raw video.)
    cameraSim.enableDrawWireframe(ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      visionSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation())
      return null;
    return visionSim.getDebugField();
  }
}
