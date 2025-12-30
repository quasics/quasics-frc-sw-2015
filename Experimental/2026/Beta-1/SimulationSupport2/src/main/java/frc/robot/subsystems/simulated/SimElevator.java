// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IElevator;

/** Simulated elevator subsystem. */
public class SimElevator extends SubsystemBase implements IElevator {
  final static double MANUAL_CONTROL_SPEED = 0.05;
  final static double SETPOINT_TOLERANCE = 0.01;

  /** Current state of the elevator. */
  private ElevatorState elevatorState = ElevatorState.IDLE;

  /** Target position for the elevator. */
  private ElevatorPosition targetPosition = ElevatorPosition.BOTTOM;

  /** Directions for the manual control handling (needed for computation). */
  private enum ManualControlDirection { UP, DOWN, UNDEFINED }

  /** Current manual direction; only valid when elevatorState is MANUAL_CONTROL. */
  private ManualControlDirection manualControlDirection = ManualControlDirection.UNDEFINED;

  /** Current height of the elevator in meters. */
  private double currentHeight;

  /** PID controller for automatic positioning. */
  private PIDController pidController = new PIDController(1, 0, 0);

  /** Constructor. */
  public SimElevator() {
    setName(NAME);
    currentHeight = getHeightForPosition(ElevatorPosition.BOTTOM);
  }

  /**
   * Static helper to get the defined height for a given elevator position.
   *
   * This is mostly redundant with IElevator.getHeightForPosition(), but that method is
   * non-static, and we need this functionality in static contexts (e.g., for setting up the
   * simulation UX).
   *
   * @param position the elevator position.
   * @return the defined height for the given position.
   */
  public static double getDefinedHeightForPosition(ElevatorPosition position) {
    return switch (position) {
      case BOTTOM -> 0.0;
      case LOW -> 1.0;
      case MEDIUM -> 2.0;
      case HIGH -> 3.0;
      case TOP -> 3.2;
      case MANUAL_CONTROL -> 0.0;
    };
  }

  @Override
  public void setTargetPosition(ElevatorPosition position) {
    targetPosition = position;
    manualControlDirection = ManualControlDirection.UNDEFINED;
    elevatorState = ElevatorState.MOVING_TO_POSITION;
  }

  @Override
  public ElevatorPosition getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void stop() {
    elevatorState = ElevatorState.IDLE;
    manualControlDirection = ManualControlDirection.UNDEFINED;
  }

  @Override
  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  @Override
  public void down() {
    manualControlDirection = ManualControlDirection.DOWN;
    elevatorState = ElevatorState.MANUAL_CONTROL;
    targetPosition = ElevatorPosition.MANUAL_CONTROL;
  }

  @Override
  public void up() {
    manualControlDirection = ManualControlDirection.UP;
    elevatorState = ElevatorState.MANUAL_CONTROL;
    targetPosition = ElevatorPosition.MANUAL_CONTROL;
  }

  @Override
  public double getCurrentHeight() {
    return currentHeight;
  }

  @Override
  public double getHeightForPosition(ElevatorPosition position) {
    return switch (position) {
      case BOTTOM, LOW, MEDIUM, HIGH, TOP -> getDefinedHeightForPosition(position);
      case MANUAL_CONTROL -> currentHeight;
    };
  }

  /** Update the simulated display elements. */
  private void updateSimulatedDisplay() {
    SimulationUxSupport.DeviceStatus status = switch (elevatorState) {
      case IDLE -> SimulationUxSupport.DeviceStatus.Idle;
      case MANUAL_CONTROL -> SimulationUxSupport.DeviceStatus.Manual;
      case MOVING_TO_POSITION ->
        (Math.abs(currentHeight - getHeightForPosition(targetPosition)) < SETPOINT_TOLERANCE)
            ? SimulationUxSupport.DeviceStatus.AtSetpoint
            : SimulationUxSupport.DeviceStatus.NotAtSetpoint;
    };

    // Update any simulated display elements here, if needed
    SimulationUxSupport.instance.updateElevator(
        currentHeight, getHeightForPosition(targetPosition), status);
  }

  @Override
  public void simulationPeriodic() {
    if (elevatorState == ElevatorState.MOVING_TO_POSITION) {
      // Simple PID control to move to target position
      double targetHeight = getHeightForPosition(targetPosition);
      double output = pidController.calculate(currentHeight, targetHeight);
      currentHeight += output * 0.02; // Simulate movement over 20ms
      if (Math.abs(currentHeight - targetHeight) < 0.01) {
        currentHeight = targetHeight;
        elevatorState = ElevatorState.IDLE;
      }
    } else if (elevatorState == ElevatorState.MANUAL_CONTROL) {
      // Simulate manual control movement, including hard stops at upper/lower limits
      if (manualControlDirection == ManualControlDirection.UP) {
        currentHeight += MANUAL_CONTROL_SPEED;
        currentHeight = Math.min(currentHeight, getHeightForPosition(ElevatorPosition.HIGH));
      } else if (manualControlDirection == ManualControlDirection.DOWN) {
        currentHeight -= MANUAL_CONTROL_SPEED;
        currentHeight = Math.max(currentHeight, getHeightForPosition(ElevatorPosition.BOTTOM));
      }
    }

    updateSimulatedDisplay();
  }
}