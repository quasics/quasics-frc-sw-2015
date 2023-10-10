// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class DriveToAprilTag extends CommandBase {
  private PhotonCamera m_camera;
  private DriveBase m_driveBase;

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new TurnToAprilTag. */
  public DriveToAprilTag(DriveBase driveBase, PhotonCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
    m_driveBase = driveBase;
    m_camera = camera;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Query the latest result from PhotonVision
    var result = m_camera.getLatestResult();

    double forwardSpeed = 0;
    double rotationSpeed = 0;

    if (result.hasTargets()) {
      // First calculate range
      double range =
              PhotonUtils.calculateDistanceToTargetMeters(
                      Constants.CAMERA_HEIGHT_METERS,
                      Constants.TARGET_HEIGHT_METERS,
                      Constants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    }

    // Use our forward/turn speeds to control the drivetrain
    m_driveBase.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
