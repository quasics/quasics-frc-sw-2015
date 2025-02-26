// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armPivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class ArmPivot extends AbstractArmPivot {
  /** Creates a new ArmPivot. */
  public ArmPivot() {
    super();
  }

  public void periodic() {
    driveArmToSetpoint();
    super.periodic();
  }

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
  public void driveArmToSetpoint() {
    if (m_angleSetpoint == null) {
      return;
    }
    final double currentAngleRadians = getPivotAngle().in(Radians);
    final double currentVelocity_radiansPerSec = getPivotVelocity();
    double pidOutput = m_armPIDController.calculate(currentAngleRadians, m_angleSetpoint.in(Radians));
    double feedForwardOutput = m_feedForward.calculate(currentAngleRadians, currentVelocity_radiansPerSec);
    double output = feedForwardOutput + pidOutput;
    m_pivot.setVoltage(output);
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

  public boolean atSetpoint() {
    return m_angleSetpoint == null // No setpoint to get to
        || m_armPIDController.atSetpoint();
  }
}
