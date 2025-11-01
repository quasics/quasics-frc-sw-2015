// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.Vision;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtTarget extends Command {
  public enum Mode {
    TargetInSight, PointAtTarget
  }

  private final AbstractDrivebase m_drivebase;
  private final Vision m_vision;
  private boolean m_inSight = false;
  private int m_fiducialId;
  private boolean m_aligned = false;
  private boolean m_finished = false;
  private final Mode m_mode;

  /** Creates a new AimAtTarget. */
  public AimAtTarget(AbstractDrivebase drivebase, Vision vision, int fiducialId, Mode mode) {
    m_drivebase = drivebase;
    m_vision = vision;
    m_fiducialId = fiducialId;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if target is in view just align, if target is not in view then turn towards
    // it
    AngularVelocity av = DegreesPerSecond.of(10);
    Angle angle = m_vision.getTargetAngleDegrees(m_fiducialId);
    if (angle == null) {
      System.out.println("cannot see target");
      m_drivebase.arcadeDrive(MetersPerSecond.of(0), DegreesPerSecond.of(40));
      return;
    }
    if (angle != null && m_mode == Mode.TargetInSight) {
      System.out.println("tag is in sight");
      m_finished = true;
    }
    if (angle != null && m_mode == Mode.PointAtTarget) {
      if (angle.gt(Degrees.of(1))) {
        m_drivebase.arcadeDrive(MetersPerSecond.of(0), DegreesPerSecond.of(8));
      }
      if (angle.lt(Degrees.of(1))) {
        av = DegreesPerSecond.of(-8);
        m_drivebase.arcadeDrive(MetersPerSecond.of(0), av);
      } else {
        m_drivebase.stop();
        m_finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
