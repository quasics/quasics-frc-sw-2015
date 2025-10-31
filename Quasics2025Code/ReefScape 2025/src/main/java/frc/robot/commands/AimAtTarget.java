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

  private final AbstractDrivebase m_drivebase;
  private final Vision m_vision;
  private boolean hasTarget = false;
  private boolean m_aligned = false;
  private int m_fiducialId;

  /** Creates a new AimAtTarget. */
  public AimAtTarget(AbstractDrivebase drivebase, Vision vision, int fiducialId) {
    m_drivebase = drivebase;
    m_vision = vision;
    m_fiducialId = fiducialId;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if target is in view just align, if target is not in view then turn towards
    // it
    Angle angle = m_vision.getTargetAngleDegrees(m_fiducialId);
    if (angle == null) {
      System.out.println("cannot see target");
    } else {
      System.out.println("tag is in sight");
      m_aligned = true;
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
    return m_aligned;
  }
}
