// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeployment;

/**
 * Sample command to use a limit switch (or similar digital input device) to
 * control the robot's actions: in this case, to stop the winch on the intake
 * when the limit switch is triggered.
 */
public class LowerIntakeAuto extends CommandBase {
  private final IntakeDeployment m_intakeDeployment;
  private final double m_speed;

  /**
   * Creates a new LowerIntakeAuto.
   * 
   * @param intakeDeployment the "intake deployment" (i.e., winch) subsystem
   * @param speed            the speed (motor %) at which the intake should be
   *                         deployed
   */
  public LowerIntakeAuto(IntakeDeployment intakeDeployment, double speed) {
    m_intakeDeployment = intakeDeployment;
    m_speed = Math.min(+1, Math.abs(speed));
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
