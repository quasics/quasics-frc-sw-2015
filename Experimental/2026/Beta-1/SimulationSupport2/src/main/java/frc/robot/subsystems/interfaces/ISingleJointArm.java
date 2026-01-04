// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;

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

  /** Returns the angle when the arm is fully extended out of the robot's frame. */
  Angle getArmOutAngle();

  /** Returns the angle when the arm is fully upright within the robot's frame. */
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
}
