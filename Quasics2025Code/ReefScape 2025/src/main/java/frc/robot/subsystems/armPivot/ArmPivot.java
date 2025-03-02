// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armPivot;

import static edu.wpi.first.units.Units.*;

public class ArmPivot extends AbstractArmPivot {
  /** Creates a new ArmPivot. */
  public ArmPivot() {
    super();

    // CODE_REVIEW: Consider calling m_armPIDController.setTolerance(<value>) to
    // establish how much error is "close enough".
  }

  public void periodic() {
    driveArmToSetpoint();
    super.periodic();
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

  public boolean atSetpoint() {
    return m_armPIDController.atSetpoint();
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
