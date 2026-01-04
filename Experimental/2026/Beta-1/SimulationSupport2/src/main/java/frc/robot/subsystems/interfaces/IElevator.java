// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

/** Interface for a simple elevator subsystem. */
public interface IElevator extends ISubsystem {
  /** The canonical name of the elevator subsystem. */
  final String SUBSYSTEM_NAME = "Elevator";

  /** The state of the elevator. */
  enum ElevatorState { IDLE, MOVING_TO_POSITION, MANUAL_CONTROL }

  /** The predefined positions for the elevator. */
  enum ElevatorPosition { BOTTOM, LOW, MEDIUM, HIGH, TOP, MANUAL_CONTROL }

  /** Stops the elevator. */
  void stop();

  /**
   * Sets the target position for the elevator.
   *
   * @param position the desired elevator position
   */
  void setTargetPosition(ElevatorPosition position);

  /** Returns the target position for the elevator. */
  ElevatorPosition getTargetPosition();

  /**
   * Gets the current state of the elevator.
   *
   * @return the current elevator state
   */
  ElevatorState getElevatorState();

  /** Moves the elevator down (under manual control). */
  void down();

  /** Moves the elevator up (under manual control). */
  void up();

  /** Gets the current height of the elevator. */
  double getCurrentHeight();

  /** Gets the height for a given position. */
  double getHeightForPosition(ElevatorPosition position);
}
