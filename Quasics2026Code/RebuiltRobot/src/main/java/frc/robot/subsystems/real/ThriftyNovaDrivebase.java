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
import frc.robot.Constants.CanBusIds.ThriftyNovaIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.ThriftyEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;

// TODO(ROBERT): Remove me
import frc.robot.subsystems.interfaces.IDrivebase;

// TODO(ROBERT): Fix Me
// public class ThriftyNovaDrivebase extends AbstractDrivebase {
public class ThriftyNovaDrivebase {
  // private final TrivialEncoder m_leftEncoder;
  // private final TrivialEncoder m_rightEncoder;

  // private final IGyro m_gyro;

  // TODO(ROBERT): Override the necessary methods

  // TODO(ROBERT): Look at SingleMotorThing by Mr. Healy
  // Used so that we can create and use the motor controllers AND encoders
  // together when we must call the ADB constructor first.
  public record ConstructionData(
      // MotorController leftMotor,
      // TrivialEncoder leftEncoder,
      // MotorController rightotor,
      // TrivialEncoder rightEncoder
      ) {
  };

  public static ConstructionData createMotorEncoders() {
    // TODO: ROBERT: these should not be member variables here. Understand why
    // TODO: ROBERT: create the rest of the encoders and motor controllers
    // ThriftyNova leftLeader = new ThriftyNova(CanBusIds.LEFT_LEADER_ID);

    // TrivialEncoder leftEncoder = new ThriftyEncoderWrapper(leftLeader,
    // Constants.wheelRadius);

    // return new ConstructionData(leftLeader, ...);
    return new ConstructionData();
  }

  /** Creates a new RealDrivebase. */
  public ThriftyNovaDrivebase() {
    this(createMotorEncoders());
  }

  public ThriftyNovaDrivebase(ConstructionData data) {
    // TODO(Robert): Understand why we need this
    // super(data.leftMotor, data.rightMotor);

    // m_leftEncoder = ???
    // m_rightEncoder = ???

    AnalogGyro gyro = new AnalogGyro(0);
    // TODO(ROBERT):
    // m_gyro = ???
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
