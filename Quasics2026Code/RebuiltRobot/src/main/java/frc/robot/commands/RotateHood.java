// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateHood extends Command {


  //private final IShooterHood m_shooterHood;
  //private final double m_pivotSpeed;
  //private final boolean m_forward;
  //private final Angle m_pivotAngle;


  /** Creates a new RotateHood. */
  public RotateHood(IShooterHood shooterHood, double pivotSpeed, boolean forward, Angle pivotAngle) {
    
    //m_shooterHood = shooterHood;
    //m_pivotSpeed = pivotSpeed;
    //m_forward = forward;
    //m_pivotAngle = pivotAngle;
    
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}