// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase implements IVision {
  /** Creates a new Vision. */
  public Vision() {
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
}
