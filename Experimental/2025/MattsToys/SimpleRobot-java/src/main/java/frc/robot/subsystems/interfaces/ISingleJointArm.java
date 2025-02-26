// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

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
  // Values defining the arm's characteristics/physics
  ////////////////////////////////////////////////////////////////////////////////////

  /**
   * Angle reported by the arm when it is fully extended from the robot's frame.
   */
  final Angle ARM_OUT_ANGLE = Degrees.of(180);

  /**
   * Angle reported by the arm when it is fully upright and within the robot's
   * frame.
   */
  final Angle ARM_UP_ANGLE = Degrees.of(90);

  /** Gearing used to drive the arm's motion. */
  final double GEARING = 5 * 5 * 3 * 4.44; // Arbitrary (but needs to be enough for
                                           // simulated physics to work)

  /**
   * Length of the arm (used for simulation, but defined here because we'd want to
   * simulate the actual hardware's physics).
   */
  final double ARM_LENGTH_METERS = 1.0; // Currently arbitrary

  /**
   * Mass of the arm (used for simulation, but defined here because we'd want to
   * simulate the actual hardware's physics).
   */
  final double ARM_MASS_KG = 4.0; // Currently arbitrary

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // Methods for controlling the arm.
  //
  ////////////////////////////////////////////////////////////////////////////////////

  /**
   * Sets the target position for the arm, to which it will be driven.
   * 
   * @param targetPosition
   */
  void setTargetPosition(Angle targetPosition);

  /** Provides a trivial (no-op) version of the arm subsystem. */
  final ISingleJointArm NULL_ARM = new ISingleJointArm() {
    @Override
    public void setTargetPosition(Angle targetPosition) {
      System.out.println("Null arm: setting target position to " + targetPosition);
      return;
    }
  };
}
