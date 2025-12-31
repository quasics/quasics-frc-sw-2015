package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;

public interface ISingleJointArm extends ISubsystem {
  /** Name of this subsystem. */
  String SUBSYSTEM_NAME = "Arm";

  /** The state of the arm. */
  enum State { IDLE, MOVING_TO_POSITION }

  void stop();

  Angle getArmMinAngle();
  Angle getArmMaxAngle();

  Angle getCurrentAngle();

  /**
   * Returns the angle when the arm is fully extended out of the robot's frame.
   *
   * @return the angle of the arm when it is extended out from the robot's
   *         centerline
   */
  Angle getArmOutAngle();

  /**
   * Returns the angle when the arm is fully upright within the robot's frame.
   *
   * @return the angle of the arm when it is extended up along the robot's
   *         centerline
   */
  Angle getArmUpAngle();

  /**
   * Sets the target position for the arm, to which it will be driven.
   *
   * @param targetPosition position to which the arm should move
   */
  void setTargetPosition(Angle targetPosition);
}
