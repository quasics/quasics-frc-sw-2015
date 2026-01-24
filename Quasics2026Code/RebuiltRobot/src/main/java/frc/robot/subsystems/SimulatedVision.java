// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Extends the basic Vision subsystem, so that we can inject simulated vision
 * data into the camera stream for that code.
 *
 * The published image streams (if enabled) follow the port order mentioned in
 * the "Camera Stream Ports" section of the PhotonVision docs. For example, a
 * single simulated camera will have its raw stream at http://localhost:1181 and
 * the processed stream at http://localhost:1182. These can also be found in the
 * CameraServer tab of Shuffleboard, like a normal camera stream.
 *
 * Note: this is implemented as a derived class so that we can restrict doing
 * this data injection solely to cases where we're running under simulation. In
 * the RobotContainer class, we'll want to check "Robot.isReal()", and then
 * either allocate the Vision base class (if it *is* a real robot), or else this
 * one (under simulation).
 */
public class SimulatedVision extends Vision {
  private VisionSystemSim m_visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;
  private final AprilTagFieldLayout m_tagLayout;
  private static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltAndymark;

  /** Creates a new SimulatedVision. */
  public SimulatedVision() {
    m_visionSim = new VisionSystemSim("main");
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }

    if (tagLayout != null) {
      m_visionSim.addAprilTags(tagLayout);
    } else {
      System.err.println("Warning: no April Tags layout loaded.");
    }

    m_tagLayout = tagLayout;

    // Set up the properties selected for our simulated camera (e.g., 640x480
    // images, etc.).
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    //
    // Set up the camera simulation object (using the "cameraProp" configuration),
    // and add it to the overall vision simulator.
    //

    // Where is the camera located, relative to the center of the robot's base?
    Translation3d robotToCameraTr = new Translation3d(0.1, 0, 0.5);
    // What is the angling of the camera, relative to the drive base?
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    // Create the overall transformation used to convert data from the robot's
    // perspective to the camera's.
    Transform3d robotToCamera = new Transform3d(robotToCameraTr, robotToCameraRot);

    // Allocate the camera simulation object.
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Enable/disable web-based aspects of the camera simulation.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(false);

    // Add the camera to the overall vision simulation framework.
    m_visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void simulationPeriodic() {
    // FIND_ME(Rylie): This is the piece that you were missing, and is why you
    // weren't seeing any of the AprilTags in the simulation of the vision data.
    // Since you weren't telling the VisionSim object, "Hey, this is where the robot
    // is", it wasn't doing the work to put the markers in place, etc.
    //
    // TODO: Add something to say where the drive base thinks it is located (since
    // we will eventually be able to drive). For now, we'll just be staying in the
    // bottom-left corner of the field, since that's what "new Pose2d()" translates
    // to....
    m_visionSim.update(new Pose2d());
  }

}
