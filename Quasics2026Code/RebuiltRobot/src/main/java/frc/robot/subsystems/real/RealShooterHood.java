// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IShooterHood;

public class RealShooterHood extends SubsystemBase implements IShooterHood {

  private final SparkMax m_hood;
  private final AbsoluteEncoder m_throughBoreEncoder;
  private final Logger m_logger = new Logger(Verbosity.Info, "RealShooterHood");

  /** Creates a new RealShooterHood. */
  public RealShooterHood() {
    m_hood = new SparkMax(SparkMaxIds.HOOD_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_hood.getAbsoluteEncoder();
  }

  @Override
  public Angle getCurrentAngle() {
    double currentAngleDegrees = m_throughBoreEncoder.getPosition() * 360;

    // FINDME(Daniel): Why are you making anything over 300 ==> 0?
    if (currentAngleDegrees > 300) {
      currentAngleDegrees = 0;
    }

    return Degrees.of(currentAngleDegrees);
  }

  /*
   * Note: this doesn't currently handle running into a limit/hard stop (which
   * should probably be looked for in periodic()).
   * 
   * FINDME(Daniel): we should really ensure that the "hard stop" case is covered.
   */
  @Override
  public void moveDown(double speed) {
    m_hood.set(Math.abs(speed));
  }

  /*
   * Note: this doesn't currently handle running into a limit/hard stop (which
   * should probably be looked for in periodic()).
   * 
   * FINDME(Daniel): we should really ensure that the "hard stop" case is covered.
   */
  @Override
  public void moveUp(double speed) {
    m_hood.set(-Math.abs(speed));
  }

  @Override
  public void stop() {
    m_hood.set(0);
  }

  @Override
  public double getCurrentAngle(){

    double currentAngleDegrees = m_throughBoreEncoder.getPosition() * 360;
    
    return currentAngleDegrees;

  }



  @Override
  public void stopHood(){

    m_hood.set(0);

  }



  @Override
  public void moveHoodIn(double speed){

    m_hood.set(Math.abs(speed));

  }



  @Override
  public void moveHoodOut(double speed){

    m_hood.set(-Math.abs(speed));

  }




  @Override
  public void periodic() {
    m_logger.log("Current Angle: " + getCurrentAngle(), Verbosity.Debug);
  }

}
