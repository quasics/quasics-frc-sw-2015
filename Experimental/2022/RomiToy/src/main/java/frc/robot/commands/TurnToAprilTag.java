// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.utils.DebugSupport.ThrottledLogger;

public class TurnToAprilTag extends CommandBase {
  final Drivetrain driveTrain;
  final PhotonVision photonVision;

  final double ANGULAR_P = 0.02;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTag(Drivetrain driveTrain, PhotonVision photonVision) {
    addRequirements(driveTrain, photonVision);
    this.driveTrain = driveTrain;
    this.photonVision = photonVision;
  }

  ThrottledLogger targetLogger = new ThrottledLogger(10);

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    photonVision.getLatestData();

    if (!photonVision.hasTargets()) {
      driveTrain.stop();
      return;
    }

    var target = photonVision.getBestTarget();
    double rotationSpeed = -turnController.calculate(target.getYaw(), 0);
    double constrainedSpeed = Math.min(Math.max(rotationSpeed, -0.5), 0.5);
    System.err.println("-- Target at: " + target.getYaw()
        + ", rotation speed: " + rotationSpeed
        + ", constrained speed: " + constrainedSpeed);
    driveTrain.arcadeDrive(0, constrainedSpeed);
  }
}
