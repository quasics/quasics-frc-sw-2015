// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;

public class SimulationDrivebase extends AbstractDrivebase {



  private final Field2d m_fieldSim = new Field2d();

  final SparkMax m_leftLeader = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  //final SparkMaxSim m_leftLeaderSim = new SparkMaxSim(m_leftLeader, );

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {}


  @Override
  protected void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds) {
      // TODO Auto-generated method stub
      

    SmartDashboard.putData("field", m_fieldSim);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
