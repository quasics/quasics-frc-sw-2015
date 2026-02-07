// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;

/** Simple interface for a single-joint arm subsystem. */
public interface ISingleJointArm extends ISubsystem {
  /** The canonical name of this subsystem. */
  String SUBSYSTEM_NAME = "Arm";

  /** The state of the arm. */
  enum State {
    /** The arm is idle (not moving). */
    IDLE,
    /** The arm is moving to a target position. */
    MOVING_TO_POSITION,
  }

  /** Stops the arm. */
  void stop();

  /** Returns the minimum angle of the arm. */
  Angle getArmMinAngle();

  /** Returns the maximum angle of the arm. */
  Angle getArmMaxAngle();

  /**
   * Returns the angle when the arm is fully extended out of the robot's frame.
   */
  Angle getArmOutAngle();

  /**
   * Returns the angle when the arm is fully upright within the robot's frame.
   */
  Angle getArmUpAngle();

  /** Returns the current angle of the arm. */
  Angle getCurrentAngle();

  /**
   * Sets the target position for the arm, to which it will be driven (via PID).
   *
   * @param targetPosition position to which the arm should move
   */
  void setTargetPosition(Angle targetPosition);

  /** Returns true if the arm is at its target position (or is idle). */
  boolean atTargetPosition();

  /** Returns the current state of the arm. */
  State getState();

  public class NullArm extends SubsystemBase implements ISingleJointArm {
    public NullArm() {
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
    public Angle getArmMinAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getArmMaxAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getArmOutAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getArmUpAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getCurrentAngle() {
      return Degrees.of(0);
    }

    @Override
    public void setTargetPosition(Angle targetPosition) {
      // No-op
    }

    @Override
    public boolean atTargetPosition() {
      return true;
    }

    @Override
    public State getState() {
      return State.IDLE;
    }
  }
}
