// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IKicker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunKicker extends Command {

  IKicker m_Kicker;
  private double m_KickerSpeed;
  // private boolean m_forward; Probably don't need

  /** Creates a new RunIntake. */
  public RunKicker(IKicker Kicker, double KickerSpeed) {

    m_Kicker = Kicker;
    m_KickerSpeed = KickerSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase) Kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Kicker.setKickSpeed(m_KickerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Kicker.setKickSpeed(m_KickerSpeed);
    System.out.println("Kicker Motor Rotation is: " + m_Kicker.getKickerPosition());
    System.out.println("Speed is: " + m_KickerSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_Kicker.stopKicker();

  }

}
