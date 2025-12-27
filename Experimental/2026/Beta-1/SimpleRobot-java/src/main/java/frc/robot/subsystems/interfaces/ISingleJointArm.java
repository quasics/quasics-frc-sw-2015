// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Defines a simple interface to control a single-joint arm the robot.
 *
 * Note: this interface assumes that the arm is completely controlled via PID,
 * with no "manual" control of the motors.
 */
public interface ISingleJointArm extends ISubsystem {
  /** Name of this subsystem. */
  String SUBSYSTEM_NAME = "Arm";

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // Methods for controlling the arm.
  //
  ////////////////////////////////////////////////////////////////////////////////////

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

  /** Provides a trivial (no-op) version of the arm subsystem. */
  final static class NullArm extends SubsystemBase implements ISingleJointArm {
    /** Constructor. */
    public NullArm() {
      System.out.println("INFO: allocating NullArm");
    }

    @Override
    public void setTargetPosition(Angle targetPosition) {
      System.out.println("Null arm: setting target position to " + targetPosition);
      return;
    }

    @Override
    public Angle getArmOutAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getArmUpAngle() {
      return Degrees.of(90);
    }
  };
}
