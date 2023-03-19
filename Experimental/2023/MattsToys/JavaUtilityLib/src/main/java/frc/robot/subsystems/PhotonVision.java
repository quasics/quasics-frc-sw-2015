// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  public final static int INVALID_TARGET_ID = -1;

  private final PhotonCamera m_camera;
  private final double m_cameraHeightMeters;
  private final double m_cameraPitchDegrees;

  /** Creates a new PhotonVision. */
  public PhotonVision(String cameraName, double cameraHeightMeters, double cameraPitchDegrees) {
    setName("PhotonVision (" + cameraName + ")");
    m_camera = new PhotonCamera(cameraName);
    m_cameraHeightMeters = cameraHeightMeters;
    m_cameraPitchDegrees = cameraPitchDegrees;
  }

  public double getCameraHeight() {
    return m_cameraHeightMeters;
  }

  public double getCameraPitch() {
    return m_cameraPitchDegrees;
  }

  /**
   * Indicates if we have a connection available to the camera.
   */
  public boolean isConnected() {
    return (m_camera != null) && m_camera.isConnected();
  }

  /**
   * Indicates if we have any targets in view.
   * 
   * @return true iff we have any targets in view.
   */
  public boolean anyTargetsAvailable() {
    if (!isConnected()) {
      return false;
    }
    PhotonPipelineResult result = m_camera.getLatestResult();
    return result.hasTargets();
  }

  /**
   * Gets the best target in view.
   * 
   * @return information on the best target in view (or null if none are
   *         available)
   */
  public PhotonTrackedTarget getBestTarget() {
    if (!isConnected()) {
      return null;
    }
    final PhotonPipelineResult result = m_camera.getLatestResult();
    if (!result.hasTargets()) {
      return null;
    }

    return result.getBestTarget();
  }

  /**
   * Returns information for the specified target.
   * 
   * @param fiducialId the desired AprilTag
   * @return information on the specified target in view (or null if not
   *         available)
   */
  public PhotonTrackedTarget getTargetInformation(int fiducialId) {
    if (!isConnected()) {
      return null;
    }
    final PhotonPipelineResult result = m_camera.getLatestResult();
    if (!result.hasTargets()) {
      return null;
    }

    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == fiducialId) {
        return target;
      }
    }

    return null;
  }

  ///////////////////////////////////////////////////////////////
  // Convenience (shortcut) functions.

  public boolean targetAvailable(int fiducialId) {
    var target = getTargetInformation(fiducialId);
    return target != null;
  }

  public int getBestTargetId() {
    var target = getBestTarget();
    if (target == null) {
      return INVALID_TARGET_ID;
    }
    return target.getFiducialId();
  }

  ///////////////////////////////////////////////////////////////
  // Stock functions from base class.

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
