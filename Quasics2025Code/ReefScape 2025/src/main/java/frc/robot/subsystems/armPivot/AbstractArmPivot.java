// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armPivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public abstract class AbstractArmPivot extends SubsystemBase {
  protected final SparkMax m_pivot;
  protected final PIDController m_armPIDController;

  protected final AbsoluteEncoder m_throughBoreEncoder;

  protected final ArmFeedforward m_feedForward;

  protected Angle m_angleSetpoint = Degrees.of(0);

  // TODO: Validate this tolerance.
  protected final Angle ANGLE_TOLERANCE_RADIANS = Degrees.of(2); // within N degrees is fine

  // 360 (degrees) / 2048 (cycles per revolution)
  // TODO: Switch this to "Angle" type.
  final double ENCODER_SCALING_FACTOR_RADIANS = Math.toRadians(360.0 / 2048.0); // TODO: test

  /** Creates a new AbstractArmPivot. */
  public AbstractArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();
    m_armPIDController = new PIDController(ArmPIDConstants.kP, ArmPIDConstants.kI, ArmPIDConstants.kD);
    m_feedForward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Through Bore Encoder Position (deg)", m_throughBoreEncoder.getPosition());
    SmartDashboard.putData("PID Controller", m_armPIDController);
  }

  public Angle getPivotAngle() {
    final double currentAngleRadians = m_throughBoreEncoder.getPosition() * ENCODER_SCALING_FACTOR_RADIANS;
    return Radians.of(currentAngleRadians);
  }

  /** @return current velocity in radians/sec */
  // TODO: Change to return RadiansPerSecond type.
  public double getPivotVelocity() {
    final double currentVelocity_radiansPerSec = m_throughBoreEncoder.getVelocity() /* in revs/min */
        * (ENCODER_SCALING_FACTOR_RADIANS / 60);
    return currentVelocity_radiansPerSec;
  }

  public double getRawPivotPosition() {
    return m_throughBoreEncoder.getPosition();
  }

  public void setArmPivotSpeed(double percentSpeed) {
    m_pivot.set(percentSpeed);
  }

  public void stop() {
    m_pivot.set(0);
  }

  public void setAngleSetpoint(Angle angle) {
    this.m_angleSetpoint = angle;
  }

  public abstract boolean atSetpoint();
}
