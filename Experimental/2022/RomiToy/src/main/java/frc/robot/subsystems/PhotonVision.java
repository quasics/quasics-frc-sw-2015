// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private final PhotonCamera camera;
  PhotonPipelineResult latestResult = null;

  /** Creates a new PhotonVision. */
  public PhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  public void getLatestData() {
    latestResult = camera.getLatestResult();
  }

  public boolean hasTargets() {
    if (latestResult == null) {
      return false;
    }

    boolean result = latestResult.hasTargets();
    return result;
  }

  public PhotonTrackedTarget getBestTarget() {
    if (latestResult == null) {
      return null;
    }

    return latestResult.getBestTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
