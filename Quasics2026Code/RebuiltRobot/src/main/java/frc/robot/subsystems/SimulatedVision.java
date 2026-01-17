// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SimulatedVision extends Vision {
  private static List<AprilTag> CUSTOM_TAGS = Arrays.asList(
      new AprilTag(0, new Pose3d(0, 0, 19.3 / 39.37, new Rotation3d())),
      new AprilTag(1, new Pose3d(0, 23.2 / 39.37, 18.6 / 39.37, new Rotation3d())));
  private static final boolean USE_REBUILT_LAYOUT = true;
  private VisionSystemSim m_visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;
  private final AprilTagFieldLayout m_tagLayout;
  private static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeAndyMark;

  /** Creates a new SimulatedVision. */
  public SimulatedVision() {
    m_visionSim = new VisionSystemSim("main");
    AprilTagFieldLayout tagLayout = null;
    if (FIELD_LAYOUT != null) {
      try {
        tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
      } catch (IOException ioe) {
        System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
        ioe.printStackTrace();
      }
    } else {
      tagLayout = new AprilTagFieldLayout(CUSTOM_TAGS, 54 * 12 / 39.37, 27 * 12 / 39.37);
    }
    m_visionSim.addAprilTags(tagLayout);

    m_tagLayout = tagLayout;
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    Translation3d robotToCameraTr = new Translation3d(0.1, 0, 0.5);
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTr, robotToCameraRot);
    m_visionSim.addCamera(cameraSim, robotToCamera);
    m_visionSim.getDebugField();
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
