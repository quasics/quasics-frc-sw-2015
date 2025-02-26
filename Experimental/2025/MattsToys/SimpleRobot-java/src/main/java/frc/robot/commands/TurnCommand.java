// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

/**
 * Simple command to turn the robot by a specified number of degrees.
 *
 * Note: this command has some bugs, since the WPILib Angle type constrains
 * values to (-180.0, +180.0] degrees. As a result, rotating past one of those
 * points will "reset" the readings, causing problems with the logic.
 *
 * This might be a good place to consider using the "OffsetGyro" type written in
 * previous years.
 */
// TODO: Fix the bug described above (caused by Angle constraining the range of
// values).
public class TurnCommand extends Command {
  final private IDrivebase m_drivebase;
  final double m_rotationInDegrees;
  final double m_rotationalSpeed;
  Angle m_stopAngle;

  /**
   * Constructor.
   *
   * @param drivebase         the drive base being controlled
   * @param rotationInDegrees the degrees to turn
   * @param rotationalSpeed   the speed to use while turning
   */
  public TurnCommand(IDrivebase drivebase, double rotationInDegrees, double rotationalSpeed) {
    m_drivebase = drivebase;
    m_rotationInDegrees = rotationInDegrees;
    m_rotationalSpeed = rotationalSpeed;

    addRequirements(m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    // Figure out when to stop (before we start moving).
    Angle startingDirection = m_drivebase.getHeading();
    Angle rotationAngle = Units.Degrees.of(m_rotationInDegrees);
    m_stopAngle = startingDirection.plus(rotationAngle);
    m_drivebase.arcadeDrive(IDrivebase.ZERO_MPS,
        AngularVelocity.ofBaseUnits(m_rotationInDegrees, Units.DegreesPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    // Note that "isEquivalent" includes a tolerence value in its computations. If
    // this is insufficient, we can switch to using the "isNear()" method, and use a
    // custom tolerance.
    if (m_drivebase.getHeading().isEquivalent(m_stopAngle)) {
      return true;
    } else {
      return false;
    }
  }
}
