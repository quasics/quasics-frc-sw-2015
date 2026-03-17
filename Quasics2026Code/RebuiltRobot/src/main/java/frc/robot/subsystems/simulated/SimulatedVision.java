// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.real.Vision;

import java.util.Collections;
import java.util.List;
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

  /**
   * Creates a new SimulatedVision.
   * 
   * @param robotToCameraTranslation translation (offsets in 3D space) from the
   *                                 robot's "center of base and forward" position
   *                                 to the camera
   * @param robotToCameraRotation3d  rotation (offset angles in 3D space) from the
   *                                 robot's "center of base and straight forward"
   *                                 position to the camera
   */
  public SimulatedVision(Translation3d robotToCameraTranslation, Rotation3d robotToCameraRotation3d) {
    super(robotToCameraTranslation, robotToCameraRotation3d);

    m_visionSim = new VisionSystemSim("main");
    if (m_tagLayout != null) {
      m_visionSim.addAprilTags(m_tagLayout);
    } else {
      System.err.println("Warning: no April Tags layout available for simulated vision.");
    }

    // Set up the properties selected for our simulated camera (e.g., 640x480
    // images, etc.).
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(480, 640, Rotation2d.fromDegrees(55));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    //
    // Set up the camera simulation object (using the "cameraProp"
    // configuration), and add it to the overall vision simulator.
    //

    // Create the overall transformation used to convert data from the robot's
    // perspective to the camera's.
    Transform3d robotToCamera = new Transform3d(robotToCameraTranslation, robotToCameraRotation3d);

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
    // FIND_ME(Rylie): Next steps here would probably be:
    // 1. Update this next line to take a report of the robot's current, actual
    // position on the field (e.g., based on odometry from the drive base, or
    // "fused" pose estimation that ties together the drive base odometry data
    // and vision estimates). This piece is what tells the camera simulation
    // stuff "Hey, pretend we're here, and inject the data into the camera for
    // what we'd see". Note that we do not want to use the estimate solely based
    // on vision here, because that will rapidly accumulate drift from errors,
    // uncertainty, etc.
    Pose2d drivePose = getDrivebasePose();
    if (drivePose != null) {
      m_visionSim.update(getDrivebasePose());
    }

    // 2. Update the simulator to reflect where the (purely) vision-based pose
    // estimate suggests that we are located.
    List<Pose2d> estimatedPoses = Collections.emptyList();
    if (latestPose2d != null) {
      estimatedPoses = Collections.singletonList(latestPose2d);
    }

    final var debugField = m_visionSim.getDebugField();
    if (estimatedPoses != null) {
      debugField.getObject("VisionEstimate").setPoses(estimatedPoses);
    } else {
      // Don't have an estimated position
      debugField.getObject("VisionEstimate").setPoses(Collections.emptyList());
    }
  }
}
