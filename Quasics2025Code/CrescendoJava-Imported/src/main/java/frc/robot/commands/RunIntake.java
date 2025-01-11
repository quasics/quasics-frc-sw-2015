// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class RunIntake extends Command {
  private final IntakeRoller m_intake;
  private final double m_intakeSpeed;
  // private final boolean m_takingIn;
  /** Creates a new RunIntake. */
  public RunIntake(IntakeRoller intake, double intakeSpeed, boolean takingIn) {
    m_intake = intake;
    // m_takingIn = takingIn;
    if (takingIn) {
      m_intakeSpeed = Math.abs(intakeSpeed);
    } else {
      m_intakeSpeed = -Math.abs(intakeSpeed);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
}
