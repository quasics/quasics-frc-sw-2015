// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.SingleSparkMaxThing;
import frc.robot.subsystems.interfaces.IIntake;


public class RealIntake extends SingleSparkMaxThing implements IIntake {


  /** Creates a new RealIntake. */
  public RealIntake() {

    super(SparkMaxIds.INTAKE_ROLLERS_ID);

  }


  @Override
  public void setRollerSpeed(double speed){

    setSpeed(speed);

  }


  @Override
  public void stopRoller(){

    stop();

  }

  public double getRollerSpeed(){

    double speed = m_motor.get();
    return speed;

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
