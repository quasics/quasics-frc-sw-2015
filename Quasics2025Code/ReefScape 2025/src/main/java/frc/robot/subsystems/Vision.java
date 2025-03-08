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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.io.IOException;
import java.util.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

// CODE_REVIEW/FIXME: Nothing is happening in this subsystem. Are you planning to make changes to add
// functionality?
public class Vision extends SubsystemBase {
  private static final boolean USE_REEFSCAPE_LAYOUT = true;
  private static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;

  /** Custom tag positions for use in the Quasics workspace. */
  private static List<AprilTag> CUSTOM_TAGS = Arrays.asList(
      new AprilTag(0, new Pose3d(0, 0, 19.3 / 39.37, new Rotation3d())),
      new AprilTag(1, new Pose3d(0, 23.2 / 39.37, 18.6 / 39.37, new Rotation3d())),
      new AprilTag(585, new Pose3d(-1, -1, 18.1, new Rotation3d())),
      new AprilTag(586, new Pose3d(26.8, -1, 20.5, new Rotation3d())));

  /**
   * sim
   * The predefined tag field layout that should be loaded (or null, if the
   * reefscape layour isn't being used).
   */
  private static final AprilTagFields FIELD_LAYOUT = USE_REEFSCAPE_LAYOUT
      ? (USE_ANDYMARK_CONFIG_FOR_REEFSCAPE ? AprilTagFields.k2025ReefscapeAndyMark
          : AprilTagFields.k2025ReefscapeWelded)
      : null // Fall back on the custom layout
  ;

  // TODO: Add the actual values for translating the robot's position to the
  // camera's position.
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());

  private PhotonCamera camera = new PhotonCamera("USB_Camera");
  private final PhotonPoseEstimator visionEstimator;

  public Vision() {
    AprilTagFieldLayout tagLayout = null;
    if (FIELD_LAYOUT != null) {
      try {
        tagLayout = AprilTagFieldLayout.loadFromResource(FIELD_LAYOUT.m_resourceFile);
      } catch (IOException ioe) {
        System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
        ioe.printStackTrace();
      }
    } else {
      tagLayout = new AprilTagFieldLayout(CUSTOM_TAGS, 54 * 12 / 39.37, 27 * 12 / 39.37);
      ;
    }

    visionEstimator = new PhotonPoseEstimator(
        tagLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
  }

  @Override
  public void periodic() {
    // var result = camera.getLatestResult();
    // SmartDashboard.putString("found target?", result.hasTargets() ? "true" :
    // "false");
  }

  @Override
  public void simulationPeriodic() {
    if (visionSim == null) {
      return;
    }

    // visionSim.update(); (get pose from sim drive base)
  }

  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;
  final boolean ENABLE_WIREFRAME_RENDERING = false;

  private void setUpSimulationSupport() {
    if (Robot.isReal()) {
      return;
    }

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(null);

    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1080, 720, Rotation2d.fromDegrees(78));
    // double check these numbers, most are placeholders
    cameraProp.setCalibError(0.0, 0.0);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(0);
    cameraProp.setLatencyStdDevMs(0);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addCamera(cameraSim, robotToCam);
    cameraSim.enableDrawWireframe(ENABLE_WIREFRAME_RENDERING);
  }

  public Field2d getDebugField() {
    if (visionSim == null) {
      return null;
    }
    return visionSim.getDebugField();
  }
}