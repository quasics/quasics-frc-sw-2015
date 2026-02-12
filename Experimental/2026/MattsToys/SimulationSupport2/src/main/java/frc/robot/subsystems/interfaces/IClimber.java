// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lightweight interface to work with a climber.
 */
public interface IClimber extends ISubsystem {
  static final String SUBSYSTEM_NAME = "Climber";

  /** Operating states for the climber. */
  enum State {
    Rising, Descending, PidControlled, Idle
  }

  /** Known/fixed positions for the climber. */
  enum Position {
    Retracted, Extended, PulledUp, DontCare
  }

  State getCurrentState();

  /** Moves the climber into the specified position. */
  void moveToPosition(Position position);

  /** Starts extending the climber (under manual control). */
  void extend();

  /** Starts retracting the climber (under manual control). */
  void retract();

  /** Stops the climber's movement. */
  void stop();

  public class NullClimber extends SubsystemBase implements IClimber {
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
