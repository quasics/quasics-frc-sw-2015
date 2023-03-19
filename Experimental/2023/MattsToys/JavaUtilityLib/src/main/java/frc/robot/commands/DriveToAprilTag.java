// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveBaseInterface;
import frc.robot.subsystems.PhotonVision;
import frc.robot.utils.MathUtils;

/**
 * Example command to drive to a given distance from an AprilTag (and end
 * pointing straight towards it).
 * 
 * Note: if the target can't be seen (or falls out of view), the command will
 * stop running. A more sophisticated version might try rotating until the
 * target is in sight (or until we've made a full sweep without finding it).
 * 
 * @see https://docs.photonvision.org/en/latest/docs/examples/aimandrange.html
 */
public class DriveToAprilTag extends CommandBase {
  final private static double ANGLE_TOLERANCE_DEGREES = 2.0;

  final private DriveBaseInterface m_driveBase;
  final private PhotonVision m_photonVision;

  final private int m_tagNum;
  final private double m_tagHeightMeters;
  final private double m_cameraHeightMeters;
  final private double m_cameraPitchRadians;
  final private double m_goalDistanceMeters;
  final private double m_goalToleranceMeters;

  private double m_lastDistance;
  private double m_lastAngle;

  final private PIDController m_forwardController;
  final private PIDController m_rotationController;

  /**
   * Constructor.
   * 
   * @param driveBase           the drive base subsystem, handling movement
   * @param photonVision        the PhotonVision wrapper subsystem, handling
   *                            tracking
   * @param tagNum              the desired AprilTag
   * @param tagHeightMeters     height of the AprilTag from the floor (in meters)
   * @param goalDistanceMeters  desired distance from the AprilTag
   * @param goalToleranceMeters tolerance that we'll accept for the distance
   * @param kP                  "kP" constant for PID control on motion
   * @param kI                  "kI" constant for PID control on motion
   * @param kD                  "kD" constant for PID control on motion
   */
  public DriveToAprilTag(DriveBaseInterface driveBase, PhotonVision photonVision, int tagNum, double tagHeightMeters,
      double goalDistanceMeters, double goalToleranceMeters,
      double kP, double kI, double kD) {
    m_driveBase = driveBase;
    m_photonVision = photonVision;
    m_tagNum = tagNum;
    m_tagHeightMeters = tagHeightMeters;
    m_goalDistanceMeters = goalDistanceMeters;
    m_goalToleranceMeters = goalToleranceMeters;
    m_cameraHeightMeters = m_photonVision.getCameraHeight();
    m_cameraPitchRadians = Units.degreesToRadians(m_photonVision.getCameraPitch());

    m_forwardController = new PIDController(kP, kI, kD);
    m_rotationController = new PIDController(kP, kI, kD);

    m_lastDistance = -1;
    m_lastAngle = -1;

    addRequirements((Subsystem) m_driveBase, photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBase.stop();

    // Reset "last" info to arbitrary values.
    m_lastAngle = -100;
    m_lastDistance = m_goalDistanceMeters + 100;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = m_photonVision.getTargetInformation(m_tagNum);
    if (target == null) {
      return;
    }

    // First calculate range
    final double range = PhotonUtils.calculateDistanceToTargetMeters(
        m_cameraHeightMeters,
        m_tagHeightMeters,
        m_cameraPitchRadians,
        Units.degreesToRadians(target.getPitch()));

    // Use the range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ range
    final double forwardSpeed = -m_forwardController.calculate(range, m_goalDistanceMeters);

    // Also calculate angular power
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    final double rotationSpeed = -m_rotationController.calculate(target.getYaw(), 0);

    // Remember the results for use in "isFinished()".
    m_lastAngle = target.getYaw();
    m_lastDistance = range;

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
    if (!m_photonVision.targetAvailable(m_tagNum)) {
      // If we can't see the target, bail out.
      return true;
    }
    if (!MathUtils.withinTolerance(m_goalDistanceMeters, m_lastDistance, m_goalToleranceMeters)) {
      // Still too far away
      return false;
    }
    if (!MathUtils.withinTolerance(0, m_lastAngle, ANGLE_TOLERANCE_DEGREES)) {
      // Not pointed correctly
      return false;
    }
    return true;
  }
}
