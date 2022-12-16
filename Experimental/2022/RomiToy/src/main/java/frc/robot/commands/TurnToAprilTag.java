// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

public class TurnToAprilTag extends CommandBase {
  final Drivetrain driveTrain;
  final PhotonVision photonVision;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTag(Drivetrain driveTrain, PhotonVision photonVision) {
    addRequirements(driveTrain, photonVision);
    this.driveTrain = driveTrain;
    this.photonVision = photonVision;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    photonVision.getLatestData();

    if (!photonVision.hasTargets()) {
      System.out.println("No targets...");
      driveTrain.stop();
    }

    var target = photonVision.getBestTarget();
    System.out.println("Target at: " + target.getYaw());
    double rotationSpeed = -turnController.calculate(target.getYaw(), 0);
    driveTrain.arcadeDrive(0, rotationSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
