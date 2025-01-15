// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {

  private static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);
  private final DifferentialDriveKinematics m_kinematics;

  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.0);
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  final SparkMax m_leftLeader = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_leftFollower =
    new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader = new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightFollower =
      new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

  public static final Distance TRACK_WIDTH_METERS = Meters.of(Constants.SallyConstants.TRACK_WIDTH);

  /** Creates a new Drivebase. */
  public Drivebase() {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    leftFollowerConfig.follow(SparkMaxIds.LEFT_LEADER_ID);
    rightFollowerConfig.follow(SparkMaxIds.RIGHT_LEADER_ID);

    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public void setSpeeds(double leftSpeed, double rightSpeed){
    // clamp speeds between -1 and 1
    leftSpeed = leftSpeed > -1 ? leftSpeed : -1;
    leftSpeed = leftSpeed < 1 ? leftSpeed : 1;
    rightSpeed = rightSpeed > -1 ? rightSpeed : -1;
    rightSpeed = rightSpeed < 1 ? rightSpeed : 1;

    m_leftLeader.set(leftSpeed);
    m_rightLeader.set(rightSpeed);
  }

  public void arcadeDrive(LinearVelocity fSpeed, AngularVelocity rSpeed){
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rSpeed)));
  }

  public void stop() {
    setSpeeds(0, 0);
  }
}
