// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

/**
 * Utility class to execute some block of code on changes to a boolean state indication.
 */
public class StateChangeExecutor {
  /** Possible "last known" states. */
  enum State {
    /** Unknown last value (e.g., before first check). */
    Unknown,
    /** Last checked state was "true". */
    True,
    /** Last checked state was "false". */
    False
  }

  /**
   * Operational mode (i.e., when the action will be executed).
   */
  public enum Mode {
    /** Execute action on any change. */
    AnyChange,
    /** Execute action when state transitions to "false". */
    GoesFalse,
    /** Execute action when state transitions to "true". */
    GoesTrue,
  }

  /** Last known state. */
  private State last = State.Unknown;
  /** Test function to use in checking state. */
  private final Supplier<Boolean> stateSupplier;
  /** Action to be taken on triggering change. */
  private final Runnable action;
  /** Operational mode (indicates triggering condition). */
  private final Mode mode;

  /**
   * Constructor.
   * @param stateSupplier test function to use in checking state
   * @param action action to be taken on triggering change
   * @param mode operational mode (indicates triggering condition)
   */
  public StateChangeExecutor(Supplier<Boolean> stateSupplier, Runnable action, Mode mode) {
    this.action = action;
    this.last = State.Unknown;
    this.stateSupplier = stateSupplier;
    this.mode = mode;
  }

  /**
   * Constructor.
   * @param stateSupplier test function to use in checking state
   * @param assumedInitialState state to assume we start in (vs. assuming "Unknown")
   * @param action action to be taken on triggering change
   * @param mode operational mode (indicates triggering condition)
   */
  public StateChangeExecutor(
      Supplier<Boolean> stateSupplier, boolean assumedInitialState, Runnable action, Mode mode) {
    this.action = action;
    this.last = assumedInitialState ? State.True : State.False;
    this.stateSupplier = stateSupplier;
    this.mode = mode;
  }

  /** Checks state and (if triggering condition met) executes the action. */
  public void check() {
    final boolean state = stateSupplier.get();
    if (state && (last == State.Unknown || (last == State.False))) {
      if (mode == Mode.AnyChange || mode == Mode.GoesTrue) {
        action.run();
      }
    } else if (!state && (last == State.Unknown || (last == State.True))) {
      if (mode == Mode.AnyChange || mode == Mode.GoesFalse) {
        action.run();
      }
    }

    last = (state ? State.True : State.False);
  }
}
