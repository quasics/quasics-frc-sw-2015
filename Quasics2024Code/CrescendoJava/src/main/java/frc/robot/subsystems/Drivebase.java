// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static edu.wpi.first.units.Units.*;
import java.lang.Math;

import frc.robot.Constants.CanBusIds.SparkMax;

public class Drivebase extends SubsystemBase {
  private final DifferentialDriveKinematics m_kinematics;
  private static final Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);


  final CANSparkMax m_leftLeader = new CANSparkMax(SparkMax.LEFT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightLeader = new CANSparkMax(SparkMax.RIGHT_LEADER_ID, MotorType.kBrushless);
  
  /** Maximum linear speed is 3 meters per second. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(3.0);

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  public static final Measure<Distance> TRACK_WIDTH_METERS = Meters.of(0.5588);


  /** Creates a new Drivebase. */
  public Drivebase() {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    setupSmartDashboard();
  }

  @Override
  public void periodic() {
  }

  public void setupSmartDashboard() {
      SmartDashboard.putData("set motor 6V", new InstantCommand(() -> setVoltages(6, 6)));
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    // clamp speeds between -1 and 1
    leftSpeed = leftSpeed > -1 ? leftSpeed : -1;
    leftSpeed = leftSpeed < 1 ? leftSpeed : 1;
    rightSpeed = rightSpeed > -1 ? rightSpeed : -1;
    rightSpeed = rightSpeed < 1 ? rightSpeed : 1;

    m_leftLeader.set(leftSpeed);
    m_rightLeader.set(rightSpeed);
  }

  public void stop() {
    setSpeeds(0, 0);
  }

  public void arcadeDrive(Measure<Velocity<Distance>> fSpeed, Measure<Velocity<Angle>> rSpeed) {
    setSpeeds(
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rSpeed)));
  }

}
