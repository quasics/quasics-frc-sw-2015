// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IIntake;

//TODO: set postitions for intake because of lack of limit switch and set an isFinished method.
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeExtension extends Command {

  private IIntake m_intakeExtenders;
  private double m_extensionSpeed;
  private boolean m_forward;

  /** Creates a new RunIntakeExtension. */
  public RunIntakeExtension(IIntake intakeExtenders, double extensionSpeed, boolean forward) {

    m_intakeExtenders = intakeExtenders;
    m_extensionSpeed = extensionSpeed;
    m_forward = forward;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) intakeExtenders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_forward) {

      m_extensionSpeed = Math.abs(m_extensionSpeed);
      m_intakeExtenders.setExtensionSpeed(m_extensionSpeed);

    } else {

      m_extensionSpeed = -Math.abs(m_extensionSpeed);
      m_intakeExtenders.setExtensionSpeed(m_extensionSpeed);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intakeExtenders.setExtensionSpeed(m_extensionSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intakeExtenders.stopExtension();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
