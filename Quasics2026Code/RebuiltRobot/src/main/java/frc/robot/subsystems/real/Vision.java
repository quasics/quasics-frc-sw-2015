// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IVision;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem implementation.
 */
public class Vision extends SubsystemBase implements IVision {
  protected static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;
  protected final AprilTagFieldLayout m_tagLayout;
  protected PhotonCamera camera = new PhotonCamera("PC_Camera");
  protected PhotonPoseEstimator photonEstimator;
  private Pose3d latestPose3d = new Pose3d();
  protected Pose2d latestPose2d = new Pose2d();
  private final Transform3d robotToCamera;

  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "Vision");

  /**
   * Constructor.
   * 
   * @param robotToCameraTranslation translation (offsets in 3D space) from the
   *                                 robot's "center of base and forward" position
   *                                 to the camera; remember that +X is forward,
   *                                 +Y is to the *left*, and +Z is up
   * @param robotToCameraRotation3d  rotation (offset angles in 3D space) from the
   *                                 robot's "center of base and straight forward"
   *                                 position to the camera
   */
  public Vision(Translation3d robotToCameraTranslation, Rotation3d robotToCameraRotation3d) {
    robotToCamera = new Transform3d(robotToCameraTranslation, robotToCameraRotation3d);

    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(
          AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException ioe) {
      System.err.println(
          "Warning: failed to load April Tags layout (" + FIELD_LAYOUT + ")");
      ioe.printStackTrace();
    }

    photonEstimator = new PhotonPoseEstimator(tagLayout,
        // FINDME(Rylie): This should ideally match the "robotToCamera"
        // configuration being used under simulation.
        //
        // FINDME(Rylie): This should ideally be coming from a robot configuration data
        // block, to give us a well-defined place to swap stuff around. (Doesn't *have*
        // to, but it's a good idea....)
        robotToCamera); // SALLY'S MEASUREMENT AS OF RIGHT NOW
    m_tagLayout = tagLayout;
  }

  private PhotonPipelineResult getLatestPipelineResults() {
    // FINDME(Rylie): You may want to take a look at what Mr. Healy is doing in the
    // "SimulationSupport" code under the "Experimental/2026/MattsToys" directory,
    // to see how he's using the new (non-deprecated) functions for getting pipeline
    // results from the PhotonCamera class.
    return camera.getLatestResult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = getLatestPipelineResults();
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    // if (m_referencePositionSupplier != null) {
    // Pose2d refPose = m_referencePositionSupplier.get();
    // if (refPose != null) {
    // photonEstimator.estimateClosestToReferencePose(result, new
    // Pose3d(refPose));
    // }
    // }

    // FINDME(Rylie): There's a subtle bug here, which you reproduce easily in the
    // simulator if you just run the "LinearSpeedCommand" from the default starting
    // position. You'll see the vision-based pose update as the robot moves, and
    // mostly follow along with the robot, then suddenly lock in place as the robot
    // continues to move away from it.
    //
    // The problem is that you're not handling the case when you *don't* have any
    // data from the pipeline (e.g., because there's no targets in view): when this
    // happens, you stop updating the estimate, but you don't "forget" the last one
    // that you computed, based on when you *did* have data. This could lead to some
    // significant errors when you're trying to use that as a part of a fused (i.e.,
    // combined odometry+vision) pose estimate.
    if (result != null) {
      visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
      if (getListOfTargetsNumber() == 1) {
        visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      if (visionEstimate.isEmpty()) {
        latestPose3d = null;
        latestPose2d = null;
        // visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
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
    var result = getLatestPipelineResults();
    hasTargets = result.hasTargets();
    return hasTargets;
  }

  public int getListOfTargetsNumber() {
    var results = getLatestPipelineResults();
    List<PhotonTrackedTarget> targets = results.getTargets();
    return targets.size();
  }

  @Override
  public List<TargetData> getTargetData() {
    var results = getLatestPipelineResults();
    List<TargetData> listTargetData = new LinkedList<>();
    List<PhotonTrackedTarget> targets = results.getTargets();
    if (!results.hasTargets()) {
      m_logger.log(Verbosity.Info, "NO TARGET DATA RETRIEVED");
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
        TargetData targetData = new TargetData(target.getFiducialId(),
            target.getYaw(), target.getPitch(), distanceToTargetPose);
        listTargetData.add(targetData);
      }
    }

    m_logger.log(Verbosity.Info, "TargetData: " + listTargetData);
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
