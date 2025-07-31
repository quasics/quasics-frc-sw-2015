// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final double m_speed;
  private final double m_durationSeconds;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new AutoDrive command.
   *
   * @param drivetrain The drivetrain subsystem this command will operate on.
   * @param speed The speed at which to drive the robot.
   * @param durationSeconds The duration in seconds to drive.
   */
  public AutoDrive(Drivetrain drivetrain, double speed,
                   double durationSeconds) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_durationSeconds = durationSeconds;
    addRequirements(m_drivetrain); // Declare that this command requires the
                                   // Drivetrain subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drivetrain.drive(m_speed,
                       m_speed); // Drive forward at the specified speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop(); // Stop the drivetrain
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(
        m_durationSeconds); // End when the duration has passed
  }
}