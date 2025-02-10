// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TurnCommand extends Command {
  final private IDrivebase m_drivebase;
  final double m_rotationInDegrees;
  final double m_rotationalSpeed;
  Angle m_stopAngle;

  /** Creates a new TurnCommand. */
  public TurnCommand(IDrivebase drivebase, double rotationInDegrees, double rotationalSpeed) {
    m_drivebase = drivebase;
    m_rotationInDegrees = rotationInDegrees;
    m_rotationalSpeed = rotationalSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebase.asSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out when to stop (before we start moving).
    Angle startingDirection = m_drivebase.getHeading();
    Angle rotationAngle = Units.Degrees.of(m_rotationInDegrees);
    m_stopAngle = startingDirection.plus(rotationAngle);
    m_drivebase.arcadeDrive(IDrivebase.ZERO_MPS,
        AngularVelocity.ofBaseUnits(m_rotationInDegrees, Units.DegreesPerSecond));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivebase.getHeading().isEquivalent(m_stopAngle)) {
      return true;
    } else {
      return false;
    }
  }
}
