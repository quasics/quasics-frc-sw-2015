// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

import java.lang.Math;

/**
 * Makes the robot turn by a certain angle (degrees, radians, whatever), at a
 * given speed.
 */
public class TurnForAngle extends Command {
  final IDrivebase m_drivebase;
  final double m_percentSpeed;
  final Angle m_turnAngle;
  Angle notfinalAngle;
  final double m_beginningAngle;

  public TurnForAngle(
      IDrivebase drivebase, double percentSpeed, Angle turnAngle) {
    m_drivebase = drivebase;

    // Angle negative5 = Degrees.of(-5);
    // Angle positive5 = Degrees.of(5);
    // if (turnAngle.in(Degrees) >= 0) {
    // m_turnAngle = turnAngle.plus(negative5);
    // } else {
    // m_turnAngle = turnAngle.plus(positive5);
    // }

    m_turnAngle = turnAngle;

    m_percentSpeed = Math.abs(percentSpeed);

    m_beginningAngle = m_drivebase.getHeading();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notfinalAngle = Degrees.of(m_drivebase.getHeading()).plus(m_turnAngle);
    System.out.println();
    System.out.println("**************                      notfinalAngle is "
        + notfinalAngle.in(Degrees));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mInOoSpercentSpeed = m_percentSpeed * -1;
    if (m_turnAngle.in(Degrees) > 0) {
      m_drivebase.setPercent(mInOoSpercentSpeed, m_percentSpeed);
    } else {
      m_drivebase.setPercent(m_percentSpeed, mInOoSpercentSpeed);
    }
    System.out.println(
        "**************                         no stop, nfA / heading ::  "
            + notfinalAngle.in(Degrees) + " / "
            + m_drivebase.getHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (0 <= m_turnAngle.in(Degrees) // positive angle
        && notfinalAngle.in(Degrees) <= m_drivebase.getHeading()) { // heading must be greater than the angle needed
      System.out.println(
          "**************                         we stop, nfA / heading ::  "
              + notfinalAngle.in(Degrees) + " / "
              + m_drivebase.getHeading());
      return true;
    }
    if (0 >= m_turnAngle.in(Degrees) // negative angle
        && notfinalAngle.in(Degrees) >= m_drivebase.getHeading()) { // heading must be less than angle needed
      System.out.println(
          "**************                         we stop, nfA / heading ::  "
              + notfinalAngle.in(Degrees) + " / "
              + m_drivebase.getHeading());
      return true;
    }

    return false;
  }
}
