// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.Constants.ArmPIDConstants;

public class ArmPivot extends SubsystemBase {
  private SparkMax m_pivot;
  private PIDController m_armPIDController;

  // CODE_REVIEW: This is only used in the constructor, so it should be local
  // there. This will make the code easier to read/maintain.

  private AbsoluteEncoder m_throughBoreEncoder;

  // CODE_REVIEW: This isn't being used at all. Do you need it? If not, then it
  // should be removed, in order to make the code easier to read/maintain.
  private ArmFeedforward m_feedForward;
  private double angleSetpointRadians;

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();
    m_armPIDController = new PIDController(ArmPIDConstants.kP, ArmPIDConstants.kI, ArmPIDConstants.kD); // TODO:
                                                                                                        // constants
                                                                                                        // need tuned
                                                                                                        // (0, 0, 0) as
                                                                                                        // of now

    m_feedForward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV); // TODO: values
                                                                                                    // require tuning
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Through Bore Encoder Position", m_throughBoreEncoder.getPosition() * 360);
    SmartDashboard.putData("PID Controller", m_armPIDController);
  }

  public double getPivotAngleRadians() {
    double currentAngleRadians = m_throughBoreEncoder.getPosition() * 360 / 2048 * Math.PI * 180; // TODO: test for
                                                                                                  // accuracy
    return currentAngleRadians;
  }

  public PIDController getPivotPIDController() {
    return m_armPIDController;
  }

  public void rotateArm(double velocity) {
    double pidOutput = m_armPIDController.calculate(getPivotAngleRadians(), angleSetpointRadians);
    double feedForwardOutput = m_feedForward.calculate(getPivotAngleRadians(),
        velocity);
    double output = feedForwardOutput + pidOutput;
    m_pivot.set(output);
  }

  public void setAngleSetpointRadians(double angleSetpoint) {
    this.angleSetpointRadians = angleSetpoint;
  }

  public double getPivotPosition() {
    return m_throughBoreEncoder.getPosition();
  }

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
