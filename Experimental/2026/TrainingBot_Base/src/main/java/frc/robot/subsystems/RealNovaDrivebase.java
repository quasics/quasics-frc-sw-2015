// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CanBusIds.PIGEON2_CAN_ID;
import static frc.robot.Constants.CanBusIds.NovaIds.LEFT_LEADER_ID;
import static frc.robot.Constants.CanBusIds.NovaIds.RIGHT_LEADER_ID;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.sensors.ThriftyEncoderWrapper;
import frc.robot.hardware.sensors.TrivialEncoder;

/**
 * Defines an implementation of the drivebase subsystem that uses real hardware,
 * based on Quasics' standard "Thrifty Nova" drivebase configuration.
 */
public class RealNovaDrivebase extends AbstractDrivebase {
  static final Distance ANDYMARK_6IN_PLACTION_DIAMETER = Inches.of(6.0);

  ThriftyNova leftLeader = new ThriftyNova(LEFT_LEADER_ID, MotorType.NEO);
  ThriftyNova rightLeader = new ThriftyNova(RIGHT_LEADER_ID, MotorType.NEO);

  TrivialEncoder leftEncoder = new ThriftyEncoderWrapper(leftLeader, ANDYMARK_6IN_PLACTION_DIAMETER);
  TrivialEncoder rightEncoder = new ThriftyEncoderWrapper(rightLeader, ANDYMARK_6IN_PLACTION_DIAMETER);

  /** The gyro/ALU that we're using for direction identification. */
  private final Pigeon2 m_rawGyro = new Pigeon2(PIGEON2_CAN_ID);

  public RealNovaDrivebase() {
  }

  @Override
  public void tankDrive(double leftPercentage, double rightPercentage) {
    leftLeader.set(leftPercentage);
    rightLeader.set(rightPercentage);
  }

  @Override
  public double getLeftDistanceMeters() {
    return leftEncoder.getPosition().in(Meters);
  }

  @Override
  public double getRightDistanceMeters() {
    return rightEncoder.getPosition().in(Meters);
  }

  @Override
  public double getHeadingInDegrees() {
    return m_rawGyro.getYaw().getValue().in(Degrees);
  }

}
