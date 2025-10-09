// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Class to drive the robot over Sim
public abstract class AbstractDrivebase extends SubsystemBase {
  // TODO: Add the items which are constant between sim and CAN
  //    - Initialize the robot
  //    - Motor commands (Hardware Abstraction Layer)

  // @Override
  // public void periodic() {
  //   // TODO: HAL - use the methods below to get the current pose and feed it into the smart dashboard
  // }

  // protected abstract TrivialEncoder getLeftEncoder_HAL();
  // protected abstract TrivialEncoder getRightEncoder_HAL();
  // protected abstract IGyro getGyro_HAL();
  // protected abstract void setMotorVoltages_HAL(double leftSpeeds, double rightSpeeds);
  // public abstract Voltage getLeftVoltage();
  // public abstract Voltage getRightVoltage();
  // protected abstract void setSpeeds_HAL(double leftSpeeds, double rightSpeeds);

  // TODO: Rename to setSpeeds_HAL 
  public abstract void driveArcade(double leftSpeeds, double rightSpeeds);

}
