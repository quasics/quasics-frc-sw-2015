package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;

public interface ISingleJointArm extends ISubsystem {
  /** The canonical name of this subsystem. */
  String SUBSYSTEM_NAME = "Arm";

  /** The state of the arm. */
  enum State { IDLE, MOVING_TO_POSITION }

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

  boolean atTargetPosition();
}
