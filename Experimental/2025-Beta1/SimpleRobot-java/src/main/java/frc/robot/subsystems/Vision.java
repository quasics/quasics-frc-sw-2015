// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  protected VisionSystemSim visionSim = new VisionSystemSim("main");

  // The simulated camera properties
  // https://docs.photonvision.org/en/v2025.0.0-alpha-0/docs/simulation/simulation-java.html#camera-simulation
  SimCameraProperties cameraProp = new SimCameraProperties();
  private PhotonCameraSim cameraSim = null;

  /** Creates a new Vision. */
  public Vision() {
    setName("Vision");
    // final TargetModel targetModel = TargetModel.kAprilTag16h5; // or
    // TargetModel.kAprilTag36h11, starting in 2024

    try {
      AprilTagFieldLayout tagLayout = AprilTagFieldLayout
          .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

      visionSim.addAprilTags(tagLayout);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout.");
      ioe.printStackTrace();
    }

    // The PhotonCamera used in the real robot code.
    PhotonCamera camera = new PhotonCamera("cameraName");

    // The simulation of this camera. Its values used in real robot code will be
    // updated.
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
    // pose, (Robot pose is considered the center of rotation at the floor level, or
    // Z = 0)...
    Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    // ...and pitched 15 degrees up.
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    // Add this camera to the vision system simulation with the given
    // robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable the raw and processed streams. These are enabled by default.
    // These streams follow the port order mentioned in Camera Stream Ports. For
    // example, a single simulated camera will have its raw stream at localhost:1181
    // and processed stream at localhost:1182, which can also be found in the
    // CameraServer tab of Shuffleboard like a normal camera stream.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // This method will be called once per scheduler run
  @Override
  public void simulationPeriodic() {
    // TODO: Get the robot pose (e.g., from drive base).
    Pose2d robotPoseMeters = new Pose2d();

    visionSim.update(robotPoseMeters);
  }
}
