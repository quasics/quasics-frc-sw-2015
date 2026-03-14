// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.logging.Logger;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.TargetPositioningUtils;

/**
 * Command to make the robot automatically turn so that its heading is aligned
 * with the center of our alliance's hub.
 */
public class AlignToHub extends Command {
  /** Tolerance for alignment. */
  final static Angle TOLERANCE = Robot.isReal()
      ? Degrees.of(4)
      : Degrees.of(1);

  /** Drivebase being controlled. */
  IDrivebase m_drivebase;
  /** Angle we want to get to. (Set in initialize().) */
  Rotation2d m_goalAngle;
  /** PID controller, used to set drivebase speeds. */
  final PIDController m_pid;
  /** Supplier used to wrap the retrieval of position from the drivebase. */
  Supplier<Pose2d> m_supplier;
  /** Logging object for debugging output. */
  Logger m_Logger = new Logger(Logger.Verbosity.Debug, "AlignToHub");

  /** Creates a new AlignToHub. */
  public AlignToHub(IDrivebase drivebase) {
    m_drivebase = drivebase;
    if (Robot.isSimulation()) {
      m_pid = new PIDController(0.004, 0.002, 0);
    } else {
      // TODO: Add tuning values for Lizzie, based on its real behavior. (This can
      // also be done by performing characterization of angular movement, and feeding
      // the data into SysId.)
      // FINDME(Rylie): Add appropriate values for Lizzie.
      m_pid = new PIDController(0.004, 0.0, 0);
    }
    m_pid.enableContinuousInput(-180, 180);

    m_supplier = () -> {
      // FINDME(Rylie): I'm pulling data from the odometry here, rather than the pose
      // estimation, because I'm getting odd results from the latter. It looks like
      // there may be some debugging required.
      return m_drivebase.getOdometryPose();
    };

    addRequirements((Subsystem) drivebase);
  }

  @Override
  public void initialize() {
    m_goalAngle = TargetPositioningUtils.getAngleToHubCenter(m_supplier.get());
  }

  @Override
  public void execute() {
    Rotation2d currentAngle = m_supplier.get().getRotation();
    Rotation2d error = m_goalAngle.minus(currentAngle);
    error = error.unaryMinus();
    double rotationPercent = m_pid.calculate(0.0, error.getDegrees());
    m_Logger.log(
        String.format(
            "Current: %3.4f, Target: %3.4f, Err: %3.4f, Power: %3.4f",
            currentAngle.getDegrees(),
            m_goalAngle.getDegrees(),
            error.getDegrees(),
            rotationPercent),
        Logger.Verbosity.Debug);
    m_drivebase.arcadeDrive(0, rotationPercent);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    Rotation2d currentAngle = m_supplier.get().getRotation();
    Rotation2d error = m_goalAngle.minus(currentAngle);
    System.out.print(
        String.format(
            "Current: %3.4f, Target: %3.4f, Err: %3.4f\n",
            currentAngle.getDegrees(),
            m_goalAngle.getDegrees(),
            error.getDegrees()));
    return error.getMeasure().abs(Degrees) <= TOLERANCE.abs(Degrees);
  }
}
