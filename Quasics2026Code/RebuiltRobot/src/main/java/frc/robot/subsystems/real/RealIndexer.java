// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.SingleSparkMaxThing;
import frc.robot.subsystems.interfaces.IIndexer;

public class RealIndexer extends SingleSparkMaxThing implements IIndexer {
  /** Creates a new RealIndexer. */
  public RealIndexer() {
    super(SparkMaxIds.INDEXER_ID);
  }


  @Override
  public void setIndexSpeed(double speed){
    setSpeed(speed);
  }


  @Override
  public void stopIndex(){
    stop();
  }
  

  public double getIndexSpeed(){
    double speed = m_motor.get();
    return speed;
  }

}
