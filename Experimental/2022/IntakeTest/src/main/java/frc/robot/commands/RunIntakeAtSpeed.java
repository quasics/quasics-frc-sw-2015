// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeAtSpeed extends CommandBase {
  private final Intake m_intake;
  private final double m_speed;

  /** Creates a new RunIntakeAtSpeed. */
  public RunIntakeAtSpeed(Intake intake, double percentSpeed) {
    addRequirements(intake);

    m_intake = intake;
    m_speed = percentSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeMotorPower(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeMotorPower(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }
}
