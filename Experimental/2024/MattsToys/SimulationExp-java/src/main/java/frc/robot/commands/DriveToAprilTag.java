// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.RobotSettings;

public class DriveToAprilTag extends Command {
  final RobotSettings.Robot m_robot;
  final VisionSubsystem m_vision;
  final AbstractDrivebase m_drivebase;
  final int m_tagId;
  final double m_tagHeightMeters;
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
  static final double P_GAIN = 0.2;
  static final double D_GAIN = 0.0;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(RobotSettings.Robot robot, VisionSubsystem vision, AbstractDrivebase drivebase, int tagId,
      Measure<Distance> tagHeight,
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

    m_forwardController = new PIDController(P_GAIN, 0, D_GAIN);
    m_turnController = new PIDController(P_GAIN, 0, D_GAIN);

    addRequirements(m_vision, m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    update();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    update();
  }

  private void update() {
    // Do we have the target in view?
    Optional<PhotonTrackedTarget> matchedTarget = m_vision.getMatchedTarget(m_tagId);
    if (!matchedTarget.isPresent()) {
      // Can't see it: turn in place and try to spot it. (Of course, there's no
      // guarantee that we'll *ever* see it, since it could be hidden by other
      // objects, or at a bad angle to the robot's camera.)
      m_drivebase.arcadeDrive(
          MetersPerSecond.of(0),
          AbstractDrivebase.MAX_ANGULAR_SPEED.divide(2));
      return;
    }

    final double rangeInMeters = getDistanceToTargetInMeters(matchedTarget.get());
    final double yaw = matchedTarget.get().getYaw();

    // We can see it: drive towards it. (Bearing in mind, of course, that we've only
    // *just* seen it, and so our line-up may not be *great*.)

    // Use this range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ range
    final double forwardSpeed = -m_forwardController.calculate(rangeInMeters, m_targetDistanceMeters)
        * AbstractDrivebase.MAX_SPEED.in(MetersPerSecond);

    // Also calculate angular power
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    final double rotationSpeed = -m_turnController.calculate(matchedTarget.get().getYaw(), 0);

    System.err
        .println("*** Range: " + rangeInMeters + "\tYaw: " + yaw + "\tForward: " + forwardSpeed + "\tRotate: "
            + rotationSpeed);

    // TODO: Finish implementing driving towards the target (matchedTarget.get()).
    m_drivebase.arcadeDrive(MetersPerSecond.of(forwardSpeed), DegreesPerSecond.of(rotationSpeed));
  }

  private double getDistanceToTargetInMeters(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        m_cameraHeightMeters,
        m_tagHeightMeters,
        m_cameraPitchRadians,
        Units.degreesToRadians(target.getPitch()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Optional<PhotonTrackedTarget> matchedTarget = m_vision.getMatchedTarget(m_tagId);
    if (matchedTarget.isEmpty()) {
      return false;
    }

    // More than 3 degrees off?
    if (Math.abs(matchedTarget.get().getYaw()) > 3) {
      return false;
    }

    final double currentDistanceInMeters = getDistanceToTargetInMeters(matchedTarget.get());
    return (currentDistanceInMeters <= m_targetDistanceMeters);
  }
}