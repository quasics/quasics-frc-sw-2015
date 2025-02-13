// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class ArmPivot extends SubsystemBase {
  // TODO: choose actual values for this in cases of simulation vs real.
  // CODE_REVIEW: This isn't being used at all. Do you need it? If not, then it
  // should be removed, in order to make the code easier to read/maintain.
  private Encoder encoder = new Encoder(4, 5);

  private SparkMax m_pivot;
  private SparkClosedLoopController m_armPIDController;

  // CODE_REVIEW: This is only used in the constructor, so it should be local
  // there. This will make the code easier to read/maintain.
  private SparkMaxConfig m_config;

  private AbsoluteEncoder m_throughBoreEncoder;

  // CODE_REVIEW: This isn't being used at all. Do you need it? If not, then it
  // should be removed, in order to make the code easier to read/maintain.
  private RelativeEncoder m_encoder;

  // CODE_REVIEW: This isn't being used at all. Do you need it? If not, then it
  // should be removed, in order to make the code easier to read/maintain.
  private ArmFeedforward m_feedForward;

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();
    m_encoder = m_pivot.getEncoder();
    m_armPIDController = m_pivot.getClosedLoopController();

    // CODE_REVIEW: You're setting up a configuration here, but not applying it to
    // the motor. Is this intentional? (I'm guessing not.)
    //
    // CODE_REVIEW: Do you know that these are reasonable PID values, or are they
    // just initial guesses? If they're just guesses, then you should probably add a
    // "TODO" comment to remind yourself to tune them later.
    m_config = new SparkMaxConfig();
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.0, 0.0, 0.0);

    // CODE_REVIEW: I'm guessing that these aren't actually the values you want to
    // use for kS, kV, and kA. (If they are, then you should probably add a comment
    // explaining why you're using these values; otherwise, you should probably add
    // a "TODO" comment to make sure that you come back to fill in "real" values.)
    m_feedForward = new ArmFeedforward(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Through Bore Encoder Position", m_throughBoreEncoder.getPosition() * 360);
  }

  public double getPivotPosition() {
    return m_throughBoreEncoder.getPosition();
  }

  // CODE_REVIEW: If this code isn't needed, then it should be removed. If it's
  // intended for later use, then it should be documented accordingly to help
  // folks understand that.
  /*
   * public void setPosition(double position) {
   * // calculate voltage FF
   * double voltageFF = m_feedForward.calculate(positionRadians, 0.0); //
   * placeholder values *THIS
   * NEEDS CHANGED* m_armPIDController.setReference(position,
   * SparkMax.ControlType.kPosition,
   * ClosedLoopSlot.kSlot0, voltageFF, ArbFFUnits.kVoltage);
   * }
   */

  public void setArmPivotSpeed(double percentSpeed) {
    m_pivot.set(percentSpeed);
  }

  public Command setArmPivotUp() {
    return this.startEnd(
        () -> {
          while (m_throughBoreEncoder.getPosition() > Constants.DesiredEncoderValues.arm90) {
            setArmPivotSpeed(-0.25);
          }
        },
        () -> {
          stop();
        });
  }

  public Command setArmPivotDown() {
    return this.startEnd(
        () -> {
          while (m_throughBoreEncoder.getPosition() < Constants.DesiredEncoderValues.arm0) {
            setArmPivotSpeed(.25); // check value and change as needed
          }
        },
        () -> {
          stop();
        });
  }

  public void stop() {
    m_pivot.set(0);
  }
}
