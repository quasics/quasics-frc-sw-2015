// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.live.SingleCameraVision;
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.CameraConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * A simulated version of the Vision subsystem, including wireframe rendering of
 * the (single) camera's data.
 *
 * The "raw" image stream will be served at http://localhost:1181/, and the
 * "processed" stream at http://localhost:1182/.
 */
public class SimulatedVision extends SingleCameraVision {
  /**
   * Handles the nuts and bolts of the actual simulation, including wireframe
   * rendering.
   */
  protected VisionSystemSim m_visionSim = new VisionSystemSim("main");

  /** The interface to control/inject simulated camera stuff. */
  private PhotonCameraSim m_cameraSim = null;

  /**
   * Constructor.
   *
   * @param config the configuration of the robot being targeted
   */
  public SimulatedVision(RobotConfig config) {
    super(config);

    //
    // Set up the camera simulation
    //

    final RobotConfigs.CameraConfig cameraConfig = config.cameras().get(0);

    // Get our first camera's data. (Will throw an exception if we don't have any,
    // but that's fair.)
    final CameraData cameraData = getCameraData().get(0);

    // Set up the camera simulation
    m_cameraSim = new PhotonCameraSim(
        cameraData.camera(),
        getCameraProperties(cameraConfig));

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

    //
    // Finish setting up the vision simulation display in the simulator.
    //

    // Add the camera to the vision system simulation with the given
    // robot-to-camera transform.
    m_visionSim.addCamera(m_cameraSim, cameraData.transform3d());

    // Add the tag layout to the vision simulation.
    if (m_tagLayout != null) {
      m_visionSim.addAprilTags(m_tagLayout);
    } else {
      System.err.println("Warning: no April Tags layout loaded.");
    }
  }

  /**
   * Returns the simulated camera properties (used to control properties like FOV,
   * resolution, etc.).
   *
   * @see <a
   *      href=
   *      "https://docs.photonvision.org/en/v2025.1.1/docs/simulation/simulation-java.html#camera-simulation">Camera
   *      simulation in PhotonVision</a>
   */
  private static SimCameraProperties getCameraProperties(CameraConfig cameraConfig) {
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(cameraConfig.imaging().width(), cameraConfig.imaging().height(),
        new Rotation2d(cameraConfig.imaging().fov()));
    cameraProp.setFPS(cameraConfig.imaging().fps());

    // Approximate detection noise with average and standard deviation error in
    // pixels.
    cameraProp.setCalibError(0.025, 0.08);

    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    return cameraProp;
  }

  // Note: this method will be called once per scheduler run
  @Override
  public void simulationPeriodic() {
    // Should be a no-op, but good practice to call the base class.
    super.simulationPeriodic();

    // Update the simulator to show where the drive base's (pure) odometry suggests
    // that we are located.
    Pose2d driveBasePoseMeters = (Pose2d) BulletinBoard.common
        .getValue(IDrivebase.ODOMETRY_KEY, Pose2d.class)
        .orElse(new Pose2d());
    m_visionSim.update(driveBasePoseMeters);

    // Update the simulator to reflect where the (purely) vision-based pose estimate
    // suggests that we are located.
    final var debugField = m_visionSim.getDebugField();
    var latestEstimates = getEstimatedPoses();
    if (!latestEstimates.isEmpty()) {
      // We're derived from SingleCameraVision, so there should only ever be 1, but
      // this keeps us safe....
      List<Pose2d> poses = new ArrayList<Pose2d>();
      for (EstimatedRobotPose estimatedRobotPose : latestEstimates) {
        poses.add(estimatedRobotPose.estimatedPose.toPose2d());
      }
      debugField.getObject("VisionEstimation").setPoses(poses);
    } else {
      debugField.getObject("VisionEstimation").setPoses();
    }

    // Update the simulator to reflect where the drivebase's (potentially composite)
    // pose estimate suggests that we are located.
    var driveBaseEstimatedPose = BulletinBoard.common
        .getValue(IDrivebase.ESTIMATED_POSE_KEY, Pose2d.class);
    driveBaseEstimatedPose.ifPresentOrElse(
        // Do this with the estimated pose from drive base (if it has some)
        est -> {
          debugField.getObject("DriveEstimation").setPose((Pose2d) est);
        },
        // If we have no estimated pose from the drive base, do this
        () -> {
          debugField.getObject("DriveEstimation").setPoses();
        });
  }
}
