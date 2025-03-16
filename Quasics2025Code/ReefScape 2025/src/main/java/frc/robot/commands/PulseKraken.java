// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRoller;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PulseKraken extends Command {
  ArmRoller m_armRoller;
  Timer m_timer;
  double m_speed;
  double m_waitTime;
  double m_intakeTime;
  boolean m_currentlyIntaking;

  /** Creates a new PulseKraken. */
  public PulseKraken(ArmRoller armRoller, double speed, double intakeTime, double waitTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armRoller = armRoller;
    m_speed = speed;
    m_intakeTime = intakeTime;
    m_waitTime = waitTime;
    m_timer = new Timer();
    m_timer.start();
    addRequirements(m_armRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_currentlyIntaking = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_currentlyIntaking);
    if (m_currentlyIntaking)
      m_armRoller.setSpeed(m_speed);
    else
      m_armRoller.stop();

    if (m_currentlyIntaking && m_timer.hasElapsed(m_intakeTime)) {
      m_currentlyIntaking = false;
      m_timer.reset();
    } else if (!m_currentlyIntaking && m_timer.hasElapsed(m_waitTime)) {
      m_currentlyIntaking = true;
      m_timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armRoller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
