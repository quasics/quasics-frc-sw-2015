// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera camera = new PhotonCamera("USB_Camera");

  List<AprilTag> tags = Arrays.asList(
      new AprilTag(0, new Pose3d(0, 0, 19.3 / 39.37, new Rotation3d())),
      new AprilTag(
          1, new Pose3d(0, 23.2 / 39.37, 18.6 / 39.37, new Rotation3d())),
      new AprilTag(585, new Pose3d(-1, -1, 18.1, new Rotation3d())),
      new AprilTag(586, new Pose3d(26.8, -1, 20.5, new Rotation3d())));

  AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(tags, 54 * 12 / 39.37, 27 * 12 / 39.37);

  Transform3d robotToCam =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
  // public PhotonPoseEstimator visionEstimator = new
  // PhotonPoseEstimator(aprilTags,
  // PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
  // robotToCam);

  public Vision() {
  }

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    if (false == results.isEmpty()) {
      var result = results.get(results.size() - 1);
      // SmartDashboard.putNumber("latency", result.getLatencyMillis());
      SmartDashboard.putString(
          "found target?", result.hasTargets() ? "true" : "false");
    } else {
      SmartDashboard.putString("found target?", "false");
    }
  }
}
