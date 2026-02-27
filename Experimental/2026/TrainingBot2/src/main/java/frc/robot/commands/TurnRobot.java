// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import frc.robot.subsystems.SimulatedDrivebase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;


/**
 * Makes the robot turn by a certain angle (degrees, radians, whatever), at a
 * given speed.
 */
public class TurnRobot extends Command {
  final AbstractDrivebase m_drivebase;
  final double m_percentSpeed;
  final Angle m_turnAngle;
  Angle m_notfinalAngle; //Angle we want to achieve
  Angle AngleToGo; //Degrees left to turn
  boolean TooSlow;
  
    /**
     * Creates a new TurnRobot.
     * 
     * Note that we need to make sure that we handle cases where (for instance) the
     * speed is negative, but the angle is positive (or vice versa). This is left as
     * an exercise for the student....
     */
    public TurnRobot(AbstractDrivebase drivebase, double percentSpeed, Angle turnAngle) { 
      m_drivebase = drivebase;
      m_turnAngle = turnAngle;
      
      double rawAngle =  turnAngle.in(Degrees);
      
      if(rawAngle < 0 && percentSpeed > 0){
        m_percentSpeed = percentSpeed * -1;
        return;
      }
      if(rawAngle > 0 && percentSpeed < 0){
        m_percentSpeed = percentSpeed * -1;
        return;
      }
      if(percentSpeed == 0){
        System.out.print(" Uh... That's too slow. ");
        m_percentSpeed = 0;
        TooSlow = true;
      }
      else{
        m_percentSpeed = percentSpeed;
      }
    
    
  
      addRequirements(m_drivebase);
    
    }
  
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      Angle Angleinstart = Degrees.of(m_drivebase.getHeadingInDegrees());
      m_notfinalAngle = Angleinstart.plus(m_turnAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Angle negativeAngle = Degrees.of(m_drivebase.getHeadingInDegrees() * -1);
    AngleToGo = m_notfinalAngle.plus(negativeAngle);
}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    Double RawAngleToGo = AngleToGo.in(Degrees);
    
    if(TooSlow = true){
      return true;
    }

  if(RawAngleToGo > 0){
    
    double rawAngle =  m_turnAngle.in(Degrees);
    double percentSpeed2 = m_percentSpeed * -1;
      
    if(rawAngle > 0){
      m_drivebase.tankDrive(m_percentSpeed, percentSpeed2);
    }
    if(rawAngle < 0){
      m_drivebase.tankDrive(percentSpeed2, m_percentSpeed);
      
    }

  return false; //Angle has not been finished/still should be turning
  }

  else{
    return true;
  }

  }
}