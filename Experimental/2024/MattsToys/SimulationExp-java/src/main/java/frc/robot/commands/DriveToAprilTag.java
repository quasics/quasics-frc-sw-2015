// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.utils.RobotSettings;
import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveToAprilTag extends Command {
  final RobotSettings.Robot m_robot;
  final VisionSubsystem m_vision;
  final AbstractDrivebase m_drivebase;
  final int m_tagId;
  final double m_tagHeightMeters;

  enum Stage { Locating, Driving }

  Stage m_currentStage = Stage.Locating;

  /**
   * Distance that the *camera* should be from the target (not the center of the
   * robot).
   */
  final double m_targetDistanceMeters;
  final double m_cameraHeightMeters;
  final double m_cameraPitchRadians;

  // PID constants should be tuned per robot
  final PIDController m_forwardController;
  final PIDController m_turnController;

  // PID constants should be tuned per robot
  static final double FORWARD_P_GAIN = 3.5;
  static final double FORWARD_I_GAIN = 0.5;
  static final double FORWARD_D_GAIN = 0.5;
  static final double ROTATE_P_GAIN = 0.05;
  static final double ROTATE_I_GAIN = 0.0;
  static final double ROTATE_D_GAIN = 0.0;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(RobotSettings.Robot robot, VisionSubsystem vision,
      AbstractDrivebase drivebase, int tagId, Measure<Distance> tagHeight,
      Measure<Distance> targetDistance) {
    this.m_robot = robot;
    this.m_vision = vision;
    this.m_drivebase = drivebase;
    this.m_tagId = tagId;
    this.m_tagHeightMeters = tagHeight.in(Meters);
    this.m_targetDistanceMeters = targetDistance.in(Meters);

    this.m_cameraHeightMeters = robot.robotToCameraTransform.getZ();
    // Negate the pitch, since we're translating from camera-relative back to
    // robot-relative values.
    this.m_cameraPitchRadians = -robot.robotToCameraTransform.getRotation().getY();

    m_forwardController = new PIDController(FORWARD_P_GAIN, FORWARD_I_GAIN, FORWARD_D_GAIN);
    m_turnController = new PIDController(ROTATE_P_GAIN, ROTATE_I_GAIN, ROTATE_D_GAIN);

    addRequirements(m_vision, m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentStage = Stage.Locating;
    m_failedSightingsWhileDriving = 0;
    update();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    update();
  }

  private String getLogPrefix() {
    return String.format(
        "%s(tag:%d, distance:%.3f):", getClass().getSimpleName(), m_tagId, m_targetDistanceMeters);
  }

  private void update() {
    final Optional<PhotonTrackedTarget> matchedTarget = m_vision.getMatchedTarget(m_tagId);
    if (m_currentStage == Stage.Locating) {
      if (matchedTarget.isPresent()) {
        // Pass on to the next stage (handled below).
        m_currentStage = Stage.Driving;
      } else {
        // Can't see it: turn in place and try to spot it. (Of course, there's no
        // guarantee that we'll *ever* see it, since it could be hidden by other
        // objects, or at a bad angle to the robot's camera.)
        m_drivebase.arcadeDrive(
            MetersPerSecond.of(0), AbstractDrivebase.MAX_ANGULAR_SPEED.divide(2));
        return;
      }
    }

    if (m_currentStage == Stage.Driving) {
      if (matchedTarget.isPresent()) {
        m_failedSightingsWhileDriving = 0;

        final double rangeInMeters = getDistanceToTargetInMeters(matchedTarget.get());
        final double yaw = matchedTarget.get().getYaw();

        // We can see it: drive towards it. (Bearing in mind, of course, that we've
        // only *just* seen it, and so our line-up may not be *great*.)

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        final double forwardCalculation =
            -m_forwardController.calculate(rangeInMeters, m_targetDistanceMeters);

        // Also calculate angular power
        final double rotationSpeed = m_turnController.calculate(matchedTarget.get().getYaw(), 0);
        System.out.println(
            String.format("> %s Range: %.3f, Yaw: %.3f, ForwardCalc: %.2f, Rotate: %.2f",
                getLogPrefix(), rangeInMeters, yaw, forwardCalculation, rotationSpeed));

        m_drivebase.arcadeDrive(forwardCalculation, rotationSpeed, false);
      } else {
        // We lost sight of it, either because we drove incorrectly, or because the
        // camera just can't track it at this point. We *could* return to the "Locating"
        // stage, but instead we'll just hold here, on the assumption that if we can't
        // see it anymore, spinning in circles isn't likely to bring it back into view.
        //
        // TODO: Decide if we want to do something different when we lose sight of the
        // target.
        System.err.println("Can't see the target");
        ++m_failedSightingsWhileDriving;
        m_drivebase.stop();
      }
    }
  }

  private double getDistanceToTargetInMeters(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(m_cameraHeightMeters, m_tagHeightMeters,
        m_cameraPitchRadians, Units.degreesToRadians(target.getPitch()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  static final double ACCEPTABLE_ERROR_ROTATION_DEGREES = 3;
  static final double ACCEPTABLE_ERROR_DISTANCE_METERS = 0.05;

  private int m_failedSightingsWhileDriving = 0;
  static final int MAX_FAILED_SIGHTINGS_WHILE_DRIVING = 5;

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Target in sight?
    Optional<PhotonTrackedTarget> matchedTarget = m_vision.getMatchedTarget(m_tagId);
    if (matchedTarget.isEmpty()) {
      if (m_currentStage == Stage.Driving
          && (m_failedSightingsWhileDriving <= MAX_FAILED_SIGHTINGS_WHILE_DRIVING)) {
        System.err.println("*** Warning: lost sight of the target for too long, giving up....");
        return true;
      }
      // OK, hope we spot it again.
      return false;
    }

    // More than N degrees off?
    final double deltaAngle = matchedTarget.get().getYaw();
    if (Math.abs(deltaAngle) > ACCEPTABLE_ERROR_ROTATION_DEGREES) {
      return false;
    }

    // More than acceptable distance away?
    final double currentDistanceInMeters = getDistanceToTargetInMeters(matchedTarget.get());
    final double deltaDistance = currentDistanceInMeters - m_targetDistanceMeters;
    if (Math.abs(deltaDistance) > ACCEPTABLE_ERROR_DISTANCE_METERS) {
      return false;
    }

    // Close enough!
    System.out.println(String.format("> %s OK, errors of %.4f meters/%.4f degrees are good enough",
        getLogPrefix(), deltaDistance, deltaAngle));
    return true;
  }
}
