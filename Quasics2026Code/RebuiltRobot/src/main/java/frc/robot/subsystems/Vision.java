// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

public class Vision extends SubsystemBase implements IVision {
  /** Creates a new Vision. */
  private static final AprilTagFields FIELD_LAYOUT = AprilTagFields.kDefaultField;
  private final AprilTagFieldLayout m_tagLayout;

  public Vision() {
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }
    m_tagLayout = tagLayout;
  }

  protected PhotonCamera camera = new PhotonCamera("camera1");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public boolean canSeeTargets() {
    boolean hasTargets;
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    return hasTargets;
  }

  public TargetData getTargetData() {
    var results = camera.getLatestResult();
    TargetData targetData = new TargetData(0, 0.0, 0.0, 0.0);
    if (!results.hasTargets()) {
      System.out.println("NO TARGET DATA RETRIEVED");
      return targetData;
    }
    if (results.hasTargets()) {
      PhotonTrackedTarget target = results.getBestTarget();
      targetData = new TargetData(target.getFiducialId(), target.getYaw(), target.getPitch(), target.getSkew());
    }
    System.out.println(targetData);
    return targetData;
  }
}
