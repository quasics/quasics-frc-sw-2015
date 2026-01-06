// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IPhotonVision;
import frc.robot.util.BulletinBoard;
import frc.robot.util.RobotConfigs;
import frc.robot.util.RobotConfigs.CameraConfig;
import frc.robot.util.RobotConfigs.RobotConfig;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Implements a subsystem that will inject simulated vision data into the setup for an actual
 * IPhotonVision-based subsystem.
 *
 * The published image streams (if enabled) follow the port order mentioned in the "Camera Stream
 * Ports" section of the PhotonVision docs. For example, a single simulated camera will have its raw
 * stream at http://localhost:1181 and the processed stream at http://localhost:1182. These can also
 * be found in the CameraServer tab of Shuffleboard, like a normal camera stream.
 *
 * Note: this is implemented as an additional subsystem, itself, simply to ensure that we have the
 * simPeriodic() function invoked on this class by the WPILib framework. We do not actually expect
 * any commands, etc., to interact with objects of this class.
 */
public class CameraSimulator extends SubsystemBase {
  /** Subsystem name. */
  static public final String SUBSYSTEM_NAME = "CameraSimulator";

  /** Determines if simulated imagery will be streamed from the library. */
  final static private boolean ENABLE_IMAGE_STREAMING = true;

  /** The primary vision object that's actually being used. */
  final private IPhotonVision m_realVision;

  /**
   * Handles the nuts and bolts of the actual simulation, including wireframe
   * rendering.
   */
  protected VisionSystemSim m_visionSim = new VisionSystemSim("main");

  /**
   * Constructor.
   *
   * @param config     the robot's configuration
   * @param realVision a PhotonVision object with the cameras into which the data
   *                   will be injected
   */
  public CameraSimulator(RobotConfig config, IPhotonVision realVision) {
    // Sanity checking parameters.
    if (config.cameras().size() != realVision.getCameraDataForSimulation().size()) {
      throw new RuntimeException("Camera data mismatch:"
          + " config has " + config.cameras().size() + " but we have "
          + realVision.getCameraDataForSimulation().size() + " cameras allocated!");
    }

    // Basic setup
    setName(SUBSYSTEM_NAME);
    m_realVision = realVision;

    // Add the tag layout to the vision simulation.
    final AprilTagFieldLayout tagLayout = m_realVision.getFieldLayoutForSimulation();
    if (tagLayout != null) {
      m_visionSim.addAprilTags(tagLayout);
    } else {
      System.err.println("Warning: no April Tags layout loaded.");
    }

    //
    // Set up simulation for each of the cameras.
    //
    for (int index = 0; index < m_realVision.getCameraDataForSimulation().size(); ++index) {
      final RobotConfigs.CameraConfig cameraConfig = config.cameras().get(index);
      final PhotonVision.CameraData cameraData =
          m_realVision.getCameraDataForSimulation().get(index);
      m_visionSim.addCamera(configureCameraSim(cameraConfig, cameraData), cameraData.transform3d());
    }
  }

  /**
   * Sets up simulation for the specified camera.
   *
   * @param cameraConfig camera configuration data
   * @param cameraData   the camera record from the underlying AbstractVision
   *                     object
   * @return the simulation controller for the camera
   */
  private PhotonCameraSim configureCameraSim(
      RobotConfigs.CameraConfig cameraConfig, PhotonVision.CameraData cameraData) {
    // Set up the camera simulation
    PhotonCameraSim cameraSim =
        new PhotonCameraSim(cameraData.camera(), getCameraProperties(cameraConfig));

    // Enable/disable the raw and processed streams. (These are enabled by default.)
    cameraSim.enableRawStream(ENABLE_IMAGE_STREAMING);
    cameraSim.enableProcessedStream(ENABLE_IMAGE_STREAMING);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    //
    // Note: This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(ENABLE_IMAGE_STREAMING);

    // Add the camera to the vision system simulation with the given
    // robot-to-camera transform.
    m_visionSim.addCamera(cameraSim, cameraData.transform3d());

    return cameraSim;
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

  //////////////////////////////////////////////////////////////////////////////
  //
  // SubsystemBase methods
  //
  //////////////////////////////////////////////////////////////////////////////

  @Override
  public void simulationPeriodic() {
    // Should be a no-op, but good practice to call the base class.
    super.simulationPeriodic();

    // Update the simulator to show where the drive base's (pure) odometry suggests
    // that we are located.
    Pose2d driveBasePoseMeters =
        (Pose2d) BulletinBoard.common.getValue(IDrivebasePlus.ODOMETRY_KEY, Pose2d.class)
            .orElse(new Pose2d());
    m_visionSim.update(driveBasePoseMeters);

    // // Update the simulator to reflect where the (purely) vision-based pose estimate
    // // suggests that we are located.
    // final var debugField = m_visionSim.getDebugField();
    // List<Pose2d> latestEstimates = m_realVision.getEstimatedPoses();
    // if (!latestEstimates.isEmpty()) {
    //   debugField.getObject("VisionEstimation").setPoses(latestEstimates);
    // } else {
    //   debugField.getObject("VisionEstimation").setPoses();
    // }

    // // Update the simulator to reflect where the drivebase's (potentially composite)
    // // pose estimate suggests that we are located.
    // var driveBaseEstimatedPose =
    //     BulletinBoard.common.getValue(IDrivebasePlus.ESTIMATED_POSE_KEY, Pose2d.class);
    // driveBaseEstimatedPose.ifPresentOrElse(
    //     // Do this with the estimated pose from drive base (if it has some)
    //     est
    //     -> { debugField.getObject("DriveEstimation").setPose((Pose2d) est); },
    //     // If we have no estimated pose from the drive base, do this
    //     () -> { debugField.getObject("DriveEstimation").setPoses(); });
  }
}
