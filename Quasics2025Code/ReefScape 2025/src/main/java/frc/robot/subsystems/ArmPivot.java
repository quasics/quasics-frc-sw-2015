// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class ArmPivot extends SubsystemBase {

  protected final SparkMax m_pivot;
  protected final PIDController m_armPIDController;

  protected final AbsoluteEncoder m_throughBoreEncoder;

  protected final ArmFeedforward m_feedForward;

  protected Angle m_angleSetpoint = null;

  // TODO: Validate this tolerance.
  protected final Angle ANGLE_TOLERANCE_RADIANS = Degrees.of(2); // within N degrees is fine

  // 360 (degrees) / 2048 (cycles per revolution)
  // TODO: Switch this to "Angle" type.
  final double ENCODER_SCALING_FACTOR_RADIANS = Math.toRadians(360.0 / 2048.0);

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_pivot.getAbsoluteEncoder();

    m_armPIDController = new PIDController(0.0, 0.0, 0.0);
    m_armPIDController.setP(10);
    m_armPIDController.setTolerance(0.5, 1);

    m_feedForward = new ArmFeedforward(0.2, 0.25, 0.0);

    final AbsoluteEncoderConfig throughBoreConfig = new AbsoluteEncoderConfig();
    throughBoreConfig.inverted(true);

    final SparkMaxConfig controllerConfig = new SparkMaxConfig();
    controllerConfig.inverted(false);
    controllerConfig.apply(throughBoreConfig);
    m_pivot.configure(controllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  static final boolean NOISY = false;

  private void driveArmToSetpoint() {
    if (m_angleSetpoint == null) {
      return;
    }
    final double currentAngleRadians = getPivotAngle().in(Radians);
    final double currentVelocity_radiansPerSec = getPivotVelocity().in(RadiansPerSecond);
    double pidOutput = m_armPIDController.calculate(currentAngleRadians, m_angleSetpoint.in(Radians));
    double feedForwardOutput = m_feedForward.calculate(currentAngleRadians, currentVelocity_radiansPerSec);
    double output = feedForwardOutput + pidOutput;
    if (NOISY) {
      System.out.printf(
          "pid: %.02f, feedforward: %.02f, output: %.02f, setpoint: %.02f\n",
          pidOutput,
          feedForwardOutput,
          output, m_angleSetpoint.in(Degrees));
    }
    m_pivot.setVoltage(output);
  }

  @Override
  public void periodic() {
    driveArmToSetpoint();

    SmartDashboard.putNumber(
        "Through Bore Encoder Position (deg)", getPivotAngle().in(Degrees));
    SmartDashboard.putNumber(
        "Through Bore Encoder Position (rad)", getPivotAngle().in(Radians));

    SmartDashboard.putNumber("Through bore encoder velocity (deg/s)", getPivotVelocity().in(DegreesPerSecond));

    SmartDashboard.putData("PID Controller", m_armPIDController);
    SmartDashboard.putNumber("armpivot p", m_armPIDController.getP());

    SmartDashboard.putBoolean("At armpivot setpoint?", m_armPIDController.atSetpoint());
  }

  public Angle getPivotAngle() {
    double currentAngleRadians = m_throughBoreEncoder.getPosition() * 2 * Math.PI;
    if (currentAngleRadians > Math.PI)
      currentAngleRadians = 0;
    return Radians.of(currentAngleRadians);
  }

  /** @return current velocity in radians/sec */
  public AngularVelocity getPivotVelocity() {
    final double currentVelocity = m_throughBoreEncoder.getVelocity() /* in revs/min */
        * 2 * Math.PI / 60; // in rad/s
    return RadiansPerSecond.of(currentVelocity);
  }

  public double getRawPivotPosition() {
    return m_throughBoreEncoder.getPosition();
  }

  public void setArmPivotSpeed(double percentSpeed) {
    m_angleSetpoint = null;
    m_pivot.set(percentSpeed);
    System.out.println(percentSpeed);
  }

  public void stop() {
    // this will also stop pid
    setArmPivotSpeed(0);
  }

  public void setAngleSetpoint(Angle angle) {
    this.m_angleSetpoint = angle;
  }

  public boolean atSetpoint() {
    return m_angleSetpoint == null // No setpoint to get to
        || m_armPIDController.atSetpoint();
  }

  // TODO: Test this.
  /*
   * public boolean atSetpoint() {
   * final Angle currentAngle = getPivotAngle();
   * Angle delta;
   * if (m_angleSetpoint.gte(currentAngle)) {
   * // setpoint >= current angle
   * delta = m_angleSetpoint.minus(currentAngle);
   * } else {
   * // setpoint < current angle
   * delta = currentAngle.minus(m_angleSetpoint);
   * }
   * return delta.lte(ANGLE_TOLERANCE_RADIANS);
   * }
   */
}
