// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class SimulationDrivebase extends AbstractDrivebase {
  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {}


  @Override
  protected void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds) {
      // TODO Auto-generated method stub
      
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
