// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armPivot;

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

  private AbsoluteEncoder m_throughBoreEncoder;

  private ArmFeedforward m_feedForward;
  private double angleSetpointRadians;

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();
    m_armPIDController = new PIDController(ArmPIDConstants.kP, ArmPIDConstants.kI, ArmPIDConstants.kD);
    m_feedForward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Through Bore Encoder Position", m_throughBoreEncoder.getPosition() * 360);
    SmartDashboard.putData("PID Controller", m_armPIDController);
  }

  public double getPivotAngleRadians() {
    // *360 (degrees) / 2048 (cycles per revolution) * pi / 180 (convert to radians)
    double currentAngleRadians = m_throughBoreEncoder.getPosition() * 360 / 2048 * Math.PI / 180; // TODO: test
    return currentAngleRadians;
  }

  // CODE_REVIEW: Don't expose the PID controller to clients; this should be
  // something managed by the subsystem (i.e., the client establishes the
  // setpoint/target value, and then the subsystem is responsible for driving to
  // that, such as via the periodic() function).
  //
  // As an example, take a look at the SimulationElevator class.
  public PIDController getPivotPIDController() {
    return m_armPIDController;
  }

  // CODE_REVIEW: You may want to consider invoking this from the periodic()
  // function, so that your commands can set the target position and then just let
  // it "automatically" move to that position.
  //
  // If you also want to be able to support manual control, you would also need to
  // establish appropriate flags, or some other way of indicating a "don't care"
  // state. (For an example, you may want to take a look at the handling of
  // AbstractElevator.TargetPosition.kDontCare.)
  public void driveArmToSetpoint(double velocity) {
    // CODE_REVIEW: PID and FF values are computed in terms of *voltages*). As a
    // result, you need to call "setVoltage()" (-12V to +12V) on the controller,
    // rather than just "set()" (which assumes you're providing a speed from -1.0 to
    // +1.0)
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

  public void stop() {
    m_pivot.set(0);
  }
}
