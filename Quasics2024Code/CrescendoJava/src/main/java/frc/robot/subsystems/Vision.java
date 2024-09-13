// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.List;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  PhotonCamera camera = new PhotonCamera("USB_Camera");

  //List<AprilTag> aprilTags = new List(new AprilTag(0, null))

  public Vision() {
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    SmartDashboard.putNumber("latency", result.getLatencyMillis());
    SmartDashboard.putString("found target?", result.hasTargets() ? "true" : "false");
  }
}
