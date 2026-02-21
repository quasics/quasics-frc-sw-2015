// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IIndexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIndexer extends Command {


  IIndexer m_index;
  private double m_indexSpeed;
  private boolean m_forward;


  /** Creates a new RunIndexer. */
  public RunIndexer(IIndexer index, double indexSpeed, boolean forward) {

    m_index = index;
    m_indexSpeed = indexSpeed;
    m_forward = forward;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase)index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_forward){

      m_indexSpeed = Math.abs(m_indexSpeed);
      m_index.setIndexSpeed(m_indexSpeed);

    } else {

      m_indexSpeed = -Math.abs(m_indexSpeed);
      m_index.setIndexSpeed(m_indexSpeed);

    }

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_index.setIndexSpeed(m_indexSpeed);

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_index.stopIndex();

  }


}
