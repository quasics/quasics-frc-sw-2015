// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

public class RealDrivebase extends AbstractDrivebase {
  // TODO: add thriftynova support. (This might be done in a derived class, or be
  // based on some information about the robot's configuration.)
  private SparkMax m_leftMotor;
  private SparkMax m_leftfollower;
  private SparkMax m_rightMotor;
  private SparkMax m_rightfollower;

  // TODO: Change these to use the encoders that are associated with the real
  // hardware (i.e., either the relative encoders that are built into the Spark
  // Max hardware, or else the functions that are built into the ThriftyNova motor
  // controller class).
  //
  // Note that Mr. Healy has updated the "TrivialEncoder" class (and some derived
  // classes) so that it can be used with Thrifty Novas (new code this year), as
  // well as the Spark Max controllers, etc.
  private Encoder m_leftEncoder = new Encoder(1, 2);
  private Encoder m_rightEncoder = new Encoder(3, 4);
  private TrivialEncoder m_mainLeftEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  private TrivialEncoder m_mainRightEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

  private DifferentialDrive m_robotDrive;
  private IGyro m_mainGyro;

  protected final IGyro getGyro() {
    return m_mainGyro;
  }

  protected final TrivialEncoder getLeftEncoder() {
    return m_mainLeftEncoder;
  }

  protected final TrivialEncoder getRightEncoder() {
    return m_mainRightEncoder;
  }

  /** Creates a new RealDrivebase. */
  public RealDrivebase() {
    // TODO: find actual SparkMax IDs, currents are placeholders.
    m_leftMotor = new SparkMax(1, MotorType.kBrushless);
    m_leftfollower = new SparkMax(2, MotorType.kBrushless);
    m_rightMotor = new SparkMax(3, MotorType.kBrushless);
    m_rightfollower = new SparkMax(4, MotorType.kBrushless);
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    AnalogGyro gyro = new AnalogGyro(0);
    m_mainGyro = IGyro.wrapGyro(gyro);

    // TODO: Configure the motor controllers on the left/right sides (e.g., ensuring
    // that "leader/follower" is set up in case a controller gets swapped out,
    // making sure that "inverted" is set correctly for each side, etc.).
  }

  public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed) {
    m_robotDrive.arcadeDrive(forwardspeed.magnitude(), turnspeed.magnitude());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
