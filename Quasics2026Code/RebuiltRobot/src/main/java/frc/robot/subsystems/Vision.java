// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;

import java.io.IOException;
import java.util.function.Supplier;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

/**
 * Vision subsystem implementation.
 */
public class Vision extends SubsystemBase implements IVision {
  private static final AprilTagFields FIELD_LAYOUT = AprilTagFields.kDefaultField;
  private final AprilTagFieldLayout m_tagLayout;
  protected PhotonCamera camera = new PhotonCamera("camera1");
  protected PhotonPoseEstimator photonEstimator;
  private Pose3d latestPose3d = new Pose3d();
  protected Pose2d latestPose2d = new Pose2d();

  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "Vision");

  /** Constructor. */
  public Vision() {
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println("Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }
    photonEstimator = new PhotonPoseEstimator(tagLayout, new Transform3d()); // should be robotToCam, update whenever
                                                                             // real camera mounted
    m_tagLayout = tagLayout;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    // if (m_referencePositionSupplier != null) {
    // Pose2d refPose = m_referencePositionSupplier.get();
    // if (refPose != null) {
    // photonEstimator.estimateClosestToReferencePose(result, new Pose3d(refPose));
    // }
    // }
    if (result != null) {
      visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
      }
    }
    if (visionEstimate.isPresent()) {
      latestPose3d = visionEstimate.get().estimatedPose;
      latestPose2d = latestPose3d.toPose2d();
    }

    // getTargetData();
  }

  public Pose2d getVisionLatestPose() {
    if (latestPose2d != null) {
      return latestPose2d;
    } else {
      return null;
    }
  }

  Supplier<Pose2d> m_referencePositionSupplier = null;

  @Override
  public void setReferencePositionSupplier(Supplier<Pose2d> supplier) {
    m_referencePositionSupplier = supplier;
  }

  @Override
  public boolean canSeeTargets() {
    boolean hasTargets;
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    return hasTargets;
  }

  public List<TargetData> getTargetData() {
    var results = camera.getLatestResult();
    List<TargetData> listTargetData = new LinkedList<>();
    List<PhotonTrackedTarget> targets = results.getTargets();
    if (!results.hasTargets()) {
      m_logger.log("NO TARGET DATA RETRIEVED", Verbosity.Info);
      return listTargetData;
    }
    if (results.hasTargets()) {
      for (PhotonTrackedTarget target : targets) {
        Optional<Pose3d> tagPose = m_tagLayout.getTagPose(target.getFiducialId());
        Pose3d tagPose3d = new Pose3d();
        if (!tagPose.isEmpty()) {
          tagPose3d = tagPose.get();
        }
        Pose2d tagPose2d = tagPose3d.toPose2d();
        double distanceToTargetPose = PhotonUtils.getDistanceToPose(latestPose2d, tagPose2d);
        TargetData targetData = new TargetData(target.getFiducialId(), target.getYaw(), target.getPitch(),
            distanceToTargetPose);
        listTargetData.add(targetData);
      }
    }

    m_logger.log("TargetData: " + listTargetData, Verbosity.Info);
    return listTargetData;
  }

  public Pose2d getDrivebasePose() {
    if (m_referencePositionSupplier != null) {
      return m_referencePositionSupplier.get();
    } else {
      return null;
    }
  }
}
