// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

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
  public static final Angle CAMERA_PITCH = Degrees.of(15); // pointed 15 degrees up
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
  protected final PhotonCamera camera;

  /** Defines the conversion from the robot's position, to ours. */
  protected final Transform3d robotToCamera;

  /** Creates a new Vision. */
  public Vision() {
    setName("Vision");

    // final TargetModel targetModel = TargetModel.kAprilTag16h5; // or
    // TargetModel.kAprilTag36h11, starting in 2024

    // Set up the relative positioning of the camera.
    Translation3d robotToCameraTrl = new Translation3d(CAMERA_X.in(Meters),
        CAMERA_Y.in(Meters), CAMERA_HEIGHT.in(Meters));
    Rotation3d robotToCameraRot = new Rotation3d(CAMERA_ROLL.in(Radians),
        -1 * CAMERA_PITCH.in(Radians),
        CAMERA_YAW.in(Radians));
    robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    // Connect to our camera. (May be a simulation.)
    camera = new PhotonCamera(CAMERA_NAME);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    super.periodic();
  }

  /**
   * Subclass to isolate simulation-specific code.
   */
  public static class SimulatedVision extends Vision {
    /**
     * Handles the nuts and bolts of the actual simulation, including wireframe
     * rendering.
     */
    protected VisionSystemSim visionSim = new VisionSystemSim("main");

    /** The interface to control/inject simulated camera stuff. */
    private PhotonCameraSim cameraSim = null;

    /** Constructor. */
    public SimulatedVision() {
      super();

      cameraSim = new PhotonCameraSim(camera, getCameraProperties());

      try {
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout
            .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        visionSim.addAprilTags(tagLayout);
      } catch (IOException ioe) {
        System.err.println("Warning: failed to load April Tags layout.");
        ioe.printStackTrace();
      }

      // Add this camera to the vision system simulation with the given
      // robot-to-camera transform.
      visionSim.addCamera(cameraSim, robotToCamera);

      // Enable the raw and processed streams. (These are enabled by default, but I'm
      // making it explicit here, so that we can easily turn them off if we decide
      // that's needed.)
      //
      // These streams follow the port order mentioned in Camera Stream Ports. For
      // example, a single simulated camera will have its raw stream at localhost:1181
      // and processed stream at localhost:1182, which can also be found in the
      // CameraServer tab of Shuffleboard like a normal camera stream.
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);

      // Enable drawing a wireframe visualization of the field to the camera streams.
      //
      // Note: This is extremely resource-intensive and is disabled by default.
      cameraSim.enableDrawWireframe(true);
    }

    /**
     * Returns the simulated camera properties (used to control properties like FOV,
     * resolution, etc.).
     * 
     * @see https://docs.photonvision.org/en/v2025.1.1/docs/simulation/simulation-java.html#camera-simulation
     */
    private SimCameraProperties getCameraProperties() {
      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, Rotation2d.fromDegrees(CAMERA_FOV_DEG));
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

      Pose2d robotPoseMeters = (Pose2d) BulletinBoard.common.getValue(IDrivebase.POSE_KEY, Pose2d.class)
          .orElse(new Pose2d());

      visionSim.update(robotPoseMeters);
    }
  }
}
