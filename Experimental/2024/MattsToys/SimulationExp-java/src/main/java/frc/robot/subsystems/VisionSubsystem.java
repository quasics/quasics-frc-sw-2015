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
   * If true, include wireframe rendering on raw video during simulation. (Note
   * that this will slow things down.)
   */
  final boolean ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO = true;

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  private final PhotonPoseEstimator m_photonEstimator;
  private final PhotonCamera m_camera;

  /** Camera mounting information, relative to the robot's centerpoint. */
  private final Transform3d kRobotToCam;

  /** Constructor. */
  public VisionSubsystem(RobotSettings.Robot robot) {
    this(robot.cameraName, robot.robotToCameraTransform);
  }

  private VisionSubsystem(String cameraName, Transform3d robotToCamera) {
    if (cameraName == null || cameraName.isEmpty()) {
      m_camera = null;
      kRobotToCam = null;
      m_photonEstimator = null;
      return;
    }

    m_camera = new PhotonCamera(cameraName);
    kRobotToCam = robotToCamera;

    // Set up the vision pose estimator
    m_photonEstimator = new PhotonPoseEstimator(
        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera,
        kRobotToCam);
    m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      setupSimulationSupport();
    }
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
