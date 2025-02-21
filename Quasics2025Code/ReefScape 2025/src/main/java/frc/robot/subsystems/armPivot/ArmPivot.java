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
    double pidOutput = m_armPIDController.calculate(
        getPivotAngleRadians(),
        angleSetpoint.in(Radians));
    double feedForwardOutput = m_feedForward.calculate(getPivotAngleRadians(),
        velocity);
    double output = feedForwardOutput + pidOutput;

    // CODE_REVIEW: PID and FF values are computed in terms of *voltages*). As a
    // result, you need to call "setVoltage()" (-12V to +12V) on the controller,
    // rather than just "set()" (which assumes you're providing a speed from -1.0 to
    // +1.0). (If you call "set()" with a voltage, you're probably going to *way*
    // overshoot your target position.)
    m_pivot.set(output);
  }
}
