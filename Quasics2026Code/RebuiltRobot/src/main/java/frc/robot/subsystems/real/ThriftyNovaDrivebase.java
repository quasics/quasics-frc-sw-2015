// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.devices.ThriftyNova;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.ThriftyEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;

public class ThriftyNovaDrivebase extends AbstractDrivebase {
  private final TrivialEncoder m_leftEncoder;
  private final TrivialEncoder m_rightEncoder;

  private final IGyro m_mainGyro;

  @Override
  protected final IGyro getGyro() {
    return m_mainGyro;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  // Used so that we can create and use the motor controllers AND encoders
  // together when we must call the ADB constructor first.
  public record ConstructionData(
      MotorController leftMotor,
      TrivialEncoder leftEncoder,
      MotorController rightMotor,
      TrivialEncoder rightEncoder) {
  };

  public static ConstructionData createMotorEncoders() {
    ThriftyNova leftLeader = new ThriftyNova(CanBusIds.LEFT_LEADER_ID);
    ThriftyNova leftFollower = new ThriftyNova(CanBusIds.LEFT_FOLLOWER_ID);
    ThriftyNova rightLeader = new ThriftyNova(CanBusIds.RIGHT_LEADER_ID);
    ThriftyNova rightFollower = new ThriftyNova(CanBusIds.RIGHT_FOLLOWER_ID);

    TrivialEncoder leftEncoder = new ThriftyEncoderWrapper(leftLeader, Constants.wheelRadius);
    TrivialEncoder rightEncoder = new ThriftyEncoderWrapper(rightLeader, Constants.wheelRadius);

    leftFollower.follow(CanBusIds.LEFT_LEADER_ID);
    rightFollower.follow(CanBusIds.RIGHT_LEADER_ID);

    return new ConstructionData(leftLeader, leftEncoder, rightLeader, rightEncoder);
  }

  /** Creates a new RealDrivebase. */
  public ThriftyNovaDrivebase() {
    this(createMotorEncoders());
  }

  public ThriftyNovaDrivebase(ConstructionData data) {
    super(data.leftMotor, data.rightMotor);

    m_leftEncoder = data.leftEncoder;
    m_rightEncoder = data.rightEncoder;

    AnalogGyro gyro = new AnalogGyro(0);
    m_mainGyro = IGyro.wrapGyro(gyro);
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
