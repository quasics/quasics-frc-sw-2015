// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeployment;

public class LowerIntakeAuto extends CommandBase {
  final IntakeDeployment m_intakeDeployment;
  final double m_speed;

  /** Creates a new LowerIntakeAuto. */
  public LowerIntakeAuto(IntakeDeployment intakeDeployment, double speed) {
    m_intakeDeployment = intakeDeployment;
    m_speed = Math.abs(speed);
    addRequirements(m_intakeDeployment);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_intakeDeployment.intakeIsDown()) {
      m_intakeDeployment.setMotorSpeed(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeDeployment.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeDeployment.intakeIsDown();
  }
}
