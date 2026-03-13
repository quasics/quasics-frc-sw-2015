// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;
import java.lang.Math;

/**
 * Makes the robot turn by a certain angle (degrees, radians, whatever), at a
 * given speed.
 */
public class TurnRobot extends Command {
  final AbstractDrivebase m_drivebase;
  final double m_percentSpeed;
  final Angle m_turnAngle;
   // final Angle m_finalAngle;   get outta here
  Angle notfinalAngle;
  final Angle m_beginningAngle;

  /**
   * Creates a new TurnRobot.
   *
   * Note that we need to make sure that we handle cases where (for instance) the
   * speed is negative, but the angle is positive (or vice versa). This is left as
   * an exercise for the student....
   */
  public TurnRobot(AbstractDrivebase drivebase, double percentSpeed, Angle turnAngle) {
    m_drivebase = drivebase;
   
    Angle negative5 = Degrees.of(-5);
    Angle positive5 = Degrees.of(5);
    if(turnAngle.in(Degrees) >= 0){
      m_turnAngle = turnAngle.plus(negative5);
    }
     else{
      m_turnAngle = turnAngle.plus(positive5);
    }

    
   
   m_percentSpeed = Math.abs(percentSpeed);


    m_beginningAngle = m_drivebase.getHeading();
 
 


    addRequirements(m_drivebase);
 
  }






  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    notfinalAngle = Degrees.of(m_drivebase.getHeadingInDegrees()).plus(m_turnAngle);
     System.out.println();
    System.out.println("**************                      notfinalAngle is " + notfinalAngle.in(Degrees));


    double mInOoSpercentSpeed = m_percentSpeed * -1;
    if (m_turnAngle.in(Degrees) > 0) {
      m_drivebase.tankDrive(m_percentSpeed, mInOoSpercentSpeed);
    } else {
      m_drivebase.tankDrive(mInOoSpercentSpeed, m_percentSpeed);




  }
}








  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
}
 




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }








  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    if(0 <= m_turnAngle.in(Degrees) && notfinalAngle.in(Degrees) <= m_drivebase.getHeadingInDegrees()){
       System.out.println("**************                         we stop, nfA / heading ::  " + notfinalAngle.in(Degrees) + " / " + m_drivebase.getHeadingInDegrees());
       return true;
    }
if(0 >= m_turnAngle.in(Degrees) && notfinalAngle.in(Degrees) >= m_drivebase.getHeadingInDegrees()){
      System.out.println("**************                         we stop, nfA / heading ::  " + notfinalAngle.in(Degrees) + " / " + m_drivebase.getHeadingInDegrees());    
      return true;
    }




    /*
    if (notfinalAngle.in(Degrees) > m_beginningAngle.in(Degrees)  &&  notfinalAngle.in(Degrees) <= m_drivebase.getHeadingInDegrees()){
      System.out.println("**************                         we stop, nfA / heading ::  " + notfinalAngle + " / " + m_drivebase.getHeadingInDegrees());  
      return true;
      }
      if (notfinalAngle.in(Degrees) < m_beginningAngle.in(Degrees) && notfinalAngle.in(Degrees) <= m_drivebase.getHeadingInDegrees() ){
        if(notfinalAngle.in(Degrees) < 0 && notfinalAngle.in(Degrees) <= m_drivebase.getHeadingInDegrees())
        System.out.println("**************                         we stop, nfA / heading ::  " + notfinalAngle + " / " + m_drivebase.getHeadingInDegrees());
          return true;
    }
    */
      return false;
  }
}















