// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;

/** Interface for a simple elevator subsystem. */
public interface IElevator extends ISubsystem {
  /** The canonical name of the elevator subsystem. */
  final String SUBSYSTEM_NAME = "Elevator";

  /** The state of the elevator. */
  enum ElevatorState {
    /** The elevator is idle. */
    IDLE,
    /** The elevator is moving to a position under PID control. */
    MOVING_TO_POSITION,
    /** The elevator is under manual control. */
    MANUAL_CONTROL,
  }

  /** The predefined positions for the elevator. */
  enum ElevatorPosition {
    /** The elevator is at the bottom (lower limit). */
    BOTTOM,
    /** The elevator is at the low position. */
    LOW,
    /** The elevator is at the medium position. */
    MEDIUM,
    /** The elevator is at the high position. */
    HIGH,
    /** The elevator is at the top (upper limit). */
    TOP,
    /**
     * The elevator is under manual control (so, it could be anywhere right
     * now).
     */
    MANUAL_CONTROL,
  }

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

  /**
   * Moves the elevator down (under manual control). It will automatically stop
   * when it reaches its bottom limit.
   */
  void down();

  /**
   * Moves the elevator up (under manual control). It will automatically stop
   * when it reaches its top limit.
   */
  void up();

  /** Gets the current height of the elevator. */
  double getCurrentHeight();

  /** Gets the height for a given position. */
  double getHeightForPosition(ElevatorPosition position);

  public class NullElevator extends SubsystemBase implements IElevator {
    public NullElevator() {
      setName(SUBSYSTEM_NAME);
    }

    @Override
    public void close() throws IOException {
      // No-op.
    }

    @Override
    public void stop() {
      // No-op.
    }

    @Override
    public void setTargetPosition(ElevatorPosition position) {
      // No-op.
    }

    @Override
    public ElevatorPosition getTargetPosition() {
      return ElevatorPosition.MANUAL_CONTROL;
    }

    @Override
    public ElevatorState getElevatorState() {
      return ElevatorState.IDLE;
    }

    @Override
    public void down() {
      // No-op.
    }

    @Override
    public void up() {
      // No-op.
    }

    @Override
    public double getCurrentHeight() {
      return 0;
    }

    @Override
    public double getHeightForPosition(ElevatorPosition position) {
      return 0;
    }
  }
}
