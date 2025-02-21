// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armPivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

/**
 * Defines a version of the ArmPivot that is primarily focused on (internalized)
 * PID usage.
 */
public class PidFocusedArmPivot extends AbstractArmPivot {
  // TODO: Validate this tolerance.
  private final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2); // within N degrees is fine

  /** Creates a new ArmPivot. */
  public PidFocusedArmPivot() {
    super();

    // Assume we don't have an initial position to get to.
    m_angleSetpoint = null;

    // Set the tolerance we're willing to accept for the arm's angle.
    m_armPIDController.setTolerance(ANGLE_TOLERANCE_RADIANS);
  }

  @Override
  public void periodic() {
    // Update PID-based control...
    driveArmToSetpoint();

    // ...and then do the base class stuff.
    super.periodic();
  }

  /** Updates the motor velocity, based on the current setpoint. */
  protected void driveArmToSetpoint() {
    if (m_angleSetpoint == null) {
      // We don't have a setpoint to drive to, so bail out.
      return;
    }

    final double currentVelocity_radiansPerSec = getPivotVelocity();
    final double currentAngle = getPivotAngleRadians();

    final double pidOutput = m_armPIDController.calculate(
        // Current position (in radians)
        currentAngle,
        // Target position
        m_angleSetpoint.in(Radians));

    final double feedForwardOutput = m_feedForward.calculate(
        // Current position (in radians)
        currentAngle,
        // Current velocity (in radians/sec)
        currentVelocity_radiansPerSec);

    // Compute new voltage to drive to the motor (and apply it)
    final double output = MathUtil.clamp(
        feedForwardOutput + pidOutput,
        -12.0, +12.0);
    m_pivot.setVoltage(output);
  }

  public boolean atSetpoint() {
    return m_angleSetpoint == null // No setpoint to get to
        || m_armPIDController.atSetpoint();
  }

  public void stop() {
    super.stop();
    m_angleSetpoint = null;
  }
}
