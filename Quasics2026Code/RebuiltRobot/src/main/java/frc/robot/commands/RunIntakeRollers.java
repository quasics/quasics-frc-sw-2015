// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeRollers extends Command {



  IIntake m_roller;
  private double m_rollerSpeed;
  private boolean m_forward;

  /** Creates a new RunIntake. */
  public RunIntakeRollers(IIntake roller, double rollerSpeed, boolean forward) {

    m_roller = roller;
    m_rollerSpeed = rollerSpeed;
    m_forward = forward;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase)roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_forward){

      m_rollerSpeed = Math.abs(m_rollerSpeed);
      m_roller.setRollerSpeed(m_rollerSpeed);

    } else {

      m_rollerSpeed = -Math.abs(m_rollerSpeed);
      m_roller.setRollerSpeed(m_rollerSpeed);

    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_roller.setRollerSpeed(m_rollerSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_roller.stopRoller();

  }

}
