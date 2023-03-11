// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveBaseInterface;

public class DriveDistance extends CommandBase {
  private final DriveBaseInterface m_drive;
  // Distance to drive in meters.
  private final double m_distance;
  private final double m_speed;
  private final boolean m_inReverse;
  private double m_targetDistance = 0;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param drive The drivetrain subsystem on which this command will run
   * @param speed The speed at which the robot will drive
   * @param meters The number of meters the robot will drive
   */
  public DriveDistance(DriveBaseInterface drive, double speed, double meters) {
    m_inReverse = (speed < 0 || meters < 0);
    final int sign = (m_inReverse ? -1 : +1);
    m_distance = Math.abs(meters) * sign;
    m_speed = Math.abs(speed) * sign;
    m_drive = drive;
    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.tankDrive(0, 0);
    m_targetDistance = m_drive.getLeftEncoderPositionMeters() + m_distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(m_speed, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double currentDistance = m_drive.getLeftEncoderPositionMeters();
    return (m_inReverse && currentDistance <= m_targetDistance)
        || (!m_inReverse && currentDistance >= m_targetDistance);
  }
}
