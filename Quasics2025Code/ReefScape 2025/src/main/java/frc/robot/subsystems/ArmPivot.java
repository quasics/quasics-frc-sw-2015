// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DesiredEncoderValues;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;


public class ArmPivot extends SubsystemBase {
  /** Creates a new ArmPivot. */
  //TODO: choose actual values for this in cases of simulation vs real.
  Encoder encoder = new Encoder(4, 5);

  SparkMax m_pivot;
  SparkClosedLoopController m_armPIDController;
  PIDController m_pid;
  SparkMaxConfig m_config;

  AbsoluteEncoder m_throughBoreEncoder;
  RelativeEncoder m_encoder;
  ArmFeedforward m_feedForward;

  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();
    m_encoder = m_pivot.getEncoder();
    m_armPIDController = m_pivot.getClosedLoopController();
    m_config = new SparkMaxConfig();
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.0, 0.0, 0.0);
    // m_feedForward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Through Bore Encoder Position", m_throughBoreEncoder.getPosition() * 360);
  }

  public double getPivotPosition() {
    return m_throughBoreEncoder.getPosition();
  }

  public void setPosition(double position) {
    m_armPIDController.setReference(position, SparkMax.ControlType.kPosition);
  }

  public void setArmPivotSpeed(double percentSpeed) {
    m_pivot.set(percentSpeed);
  }

  public Command setArmPivotUp() {
    return this.startEnd(
      ()-> {
        while (m_throughBoreEncoder.getPosition() > Constants.DesiredEncoderValues.arm0) {
          setArmPivotSpeed(-0.25);
        }
      }, 
      ()-> {
        stop();
      });
  }

  public Command setArmPivotDown() {
    return this.startEnd(
      ()-> {
        while (m_throughBoreEncoder.getPosition() < Constants.DesiredEncoderValues.arm90) {
          setArmPivotSpeed(.25); // check value and change as needed
        }
      }, 
      ()-> { 
        stop();
      });
  }

  public void stop() {
    m_pivot.set(0);
  }
}
