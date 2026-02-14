// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class TimedRunIntake extends Command {
  private final IntakeRoller m_intake;
  private final double m_intakeSpeed;
  Timer m_timer = new Timer();
  private final Time m_time;
  /** Creates a new TimedRunIntake. */
  public TimedRunIntake(IntakeRoller intake, double intakeSpeed, Time time, boolean takingIn) {
    m_time = time;
    m_intake = intake;
    if (takingIn) {
      m_intakeSpeed = -Math.abs(intakeSpeed);
    } else {
      m_intakeSpeed = Math.abs(intakeSpeed);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_intake.setRollerSpeed(m_intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setRollerSpeed(m_intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time.in(Seconds));
  }
}
