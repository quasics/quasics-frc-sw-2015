// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.io.IOException;
import java.util.function.Supplier;
import java.util.List;
import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// CODE_REVIEW/FIXME: Nothing is happening in this subsystem. Are you planning to make changes to add
// functionality?
public class Vision extends SubsystemBase {
  record TargetData(int id, Angle angle, Distance distance) {
  }

  private static final boolean USE_REEFSCAPE_LAYOUT = true;
  private static final boolean USE_ANDYMARK_CONFIG_FOR_REEFSCAPE = false;
  private Optional<EstimatedRobotPose> latestPose = Optional.empty();

  /** Custom tag positions for use in the Quasics workspace. */
  private static List<AprilTag> CUSTOM_TAGS = Arrays.asList(
      new AprilTag(0, new Pose3d(0, 0, 19.3 / 39.37, new Rotation3d())),
      new AprilTag(1, new Pose3d(0, 23.2 / 39.37, 18.6 / 39.37, new Rotation3d())),
      new AprilTag(585, new Pose3d(-1, -1, 18.1, new Rotation3d())),
      new AprilTag(586, new Pose3d(26.8, -1, 20.5, new Rotation3d())));

  /**
   * sim
   * The predefined tag field layout that should be loaded (or null, if the
   * reefscape layout isn't being used).
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
  public final PhotonPoseEstimator visionEstimator;
  private Supplier<Pose2d> poseSupplier;
  private Pose2d pose;
  private Pose3d fieldPose;
  private Pose3d robotPose3d;
  private final AprilTagFieldLayout m_tagLayout;
  private PhotonTrackedTarget target;
  private Pose2d simPose;

  public Vision(Supplier<Pose2d> pSupplier) {
    poseSupplier = pSupplier;
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
        tagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    m_tagLayout = tagLayout;
    setUpSimulationSupport();
  }

  @Override
  public void periodic() {
    /*
     * if (Robot.isSimulation()) {
     * latestPose.ifPresentOrElse(
     * est ->
     * getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.
     * toPose2d()),
     * () -> {
     * getDebugField().getObject("VisionEstimation").setPoses();
     * });
     * }
     * }
     */
    findTargets();
    if (target != null) {
      if (m_tagLayout.getTagPose(target.getFiducialId()).isPresent()) {
        robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
            m_tagLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
        // System.out.println("Target ID: " + target.getFiducialId() + " Target Yaw: " +
        // target.getYaw() + " Target Pitch: "
        // + target.getPitch());
      }
    }

    // TODO: "Verbose" mode (allow us to turn off debugging output if we want to
    // debug something else)
    if (Robot.isSimulation()) {
      if (simPose != null) {
        simPose = robotPose3d.toPose2d();
      }
      // System.out.println(simPose);
    }
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    // SmartDashboard.putString("found target?", result.hasTargets() ? "true" :
    // "false");

    // TODO: "Verbose" mode (allow us to turn off debugging output if we want to
    // debug something else)
    /*
     * if (Robot.isSimulation()) {
     * Pose2d simPose = robotPose3d.toPose2d();
     * System.out.println(simPose);
     * }
     */
    // simulationPeriodic();
  }

  @Override
  public void simulationPeriodic() {
    if (visionSim == null) {
      return;
    }
    updateEstimatedGlobalPose();
    // System.out.println(getFieldRobotPose());
    // updateEstimatedPoseToCamera();
  }

  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;
  final boolean ENABLE_WIREFRAME_RENDERING = true;

  private void setUpSimulationSupport() {
    if (Robot.isReal()) {
      return;
    }

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(m_tagLayout);
    getDebugField();

    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 320, Rotation2d.fromDegrees(78));
    // double check these numbers, most are placeholders
    cameraProp.setCalibError(0, 0.0);
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

  public void resetSimPose(Pose2d pose) {
    if (visionSim != null) {
      visionSim.resetRobotPose(pose);
    }
  }

  private Pose2d getPose() {
    pose = poseSupplier.get();
    return pose;
  }

  private void updateEstimatedGlobalPose() {
    if (camera == null) {
      return;
    }
    // Fixme: Throwing out the return value
    // Suggestion: Variable naming - "OdometryPose" vs "VisionEstimatedPose"?

    getPose();
    // System.out.println(pose);
    visionSim.update(pose);
  }

  /*
   * public Pose3d getFieldRobotPose() {
   * if (target == null) {
   * return null;
   * }
   * if (m_tagLayout.getTagPose(target.getFiducialId()).isPresent()) {
   * fieldPose =
   * PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
   * m_tagLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
   * }
   * return fieldPose;
   * }
   */

  private Optional<EstimatedRobotPose> updateEstimatedPoseToCamera() {
    if (pose != null) {
      visionEstimator.setLastPose(pose);
      visionEstimator.setReferencePose(pose);
    }

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    for (PhotonPipelineResult result : results) {
      lastEstimatedPose = visionEstimator.update(result);
    }
    return lastEstimatedPose;
  }

  public void findTargets() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    } else {
      for (PhotonPipelineResult result : results) {
        latestPose = visionEstimator.update(result);
        boolean hasTargets = result.hasTargets();
        if (hasTargets == false) {
          return;
        } else {
          List<PhotonTrackedTarget> targets = result.getTargets();
          target = result.getBestTarget();
        }
        SmartDashboard.putString("found target?", result.hasTargets() ? "true" : "false");
      }
    }
  }

  public int getBestTargetID() {
    findTargets();
    int id = target.getFiducialId();
    return id;
  }

  public Angle getTargetAngleDegrees(int id) {
    Angle angle = null;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return null;
    }
    for (PhotonPipelineResult result : results) {
      boolean hasTargets = result.hasTargets();
      if (hasTargets == false) {
        return null;
      } else {
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
          if (id == target.getFiducialId()) {
            angle = Degrees.of(target.getYaw());
          }
        }
      }
    }
    return angle;
  }

  public double getTargetRange(int id) {
    double range = 0.0;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return 0.0;
    }
    for (PhotonPipelineResult result : results) {
      boolean hasTargets = result.hasTargets();
      if (hasTargets == false) {
        return 0.0;
      } else {
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
          if (id == target.getFiducialId()) {
            range = PhotonUtils.calculateDistanceToTargetMeters(0.0, 0.17, 0.0, target.getPitch()); // height changes
                                                                                                    // based on target
          }
        }
      }
    }
    return range;
  }
}