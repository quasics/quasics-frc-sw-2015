// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;

/**
 * Lightweight interface to work with a climber.
 *
 * Note: this interface is intentionally minimal and does not include any
 * methods for querying the climber's position or other state; this is because
 * the climber's exact position is not critical. However, it *is* assumed that
 * the climber has some sort of position feedback (e.g., an encoder) that allows
 * it to move to defined positions under PID control.
 */
public interface IClimber extends ISubsystem {
  /** Subsystem name. */
  static final String SUBSYSTEM_NAME = "Climber";

  /** Operating states for the climber. */
  enum State {
    /** Climber is rising under manual control. */
    Rising,
    /** Climber is descending under manual control. */
    Descending,
    /** Climber is moving under PID control to a defined position. */
    PidControlled,
    /** Climber is not moving. */
    Idle
  }

  /** Known/fixed positions for the climber. */
  enum Position {
    /** Fully retracted position. */
    Retracted,
    /** Fully extended position. */
    Extended,
    /**
     * Position should be lifting the robot off the ground (if climbers are
     * hooked into place on the rung).
     */
    PulledUp,
    /**
     * Don't care about the position; used for manual control and for cases
     * where we don't have a defined position to move to.
     */
    DontCare
  }

  /** Returns the current operating state of the climber. */
  State getCurrentState();

  /** Moves the climber into the specified position. */
  void moveToPosition(Position position);

  /** Starts extending the climber (under manual control). */
  void extend();

  /** Starts retracting the climber (under manual control). */
  void retract();

  /** Stops the climber's movement. */
  void stop();

  /** Trivial implementation of the IClimber interface. */
  public class NullClimber extends SubsystemBase implements IClimber {
    public NullClimber() {
      setName("Null" + SUBSYSTEM_NAME);
    }

    @Override
    public void close() throws IOException {
    }

    @Override
    public void moveToPosition(Position position) {
    }

    @Override
    public void extend() {
    }

    @Override
    public void retract() {
    }

    @Override
    public void stop() {
    }

    @Override
    public State getCurrentState() {
      return State.Idle;
    }
  }
}
