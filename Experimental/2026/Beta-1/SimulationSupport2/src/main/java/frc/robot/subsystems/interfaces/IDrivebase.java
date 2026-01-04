// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/** Interface for a drivebase subsystem. */
public interface IDrivebase extends ISubsystem {
  /** Canonical name for the subsystem. */
  String SUBSYSTEM_NAME = "Drivebase";

  /** Returns the maximum linear speed allowed for the robot (under direct control). */
  LinearVelocity getMaxLinearSpeed();

  /** Returns the maximum rotational speed allowed for the robot (under direct control). */
  AngularVelocity getMaxRotationalSpeed();

  /** Returns the robot's kinematics. */
  DifferentialDriveKinematics getKinematics();

  /**
   * "Classic" arcade-style driving, based on percentages. (Note: operates
   * directly; no PID.)
   *
   * @param forward  forward speed (-1.0 to 1.0)
   * @param rotation rotation rate (-1.0 to 1.0)
   */
  void driveArcade(double forward, double rotation);

  /**
   * "Classic" tank-style driving, based on percentages. (Note: operates directly;
   * no PID.)
   *
   * @param leftSpeed  speed for the left side (-1.0 to 1.0)
   * @param rightSpeed speed for the right side (-1.0 to 1.0)
   */
  void driveTank(double leftSpeed, double rightSpeed);

  /** Stops all motion of the drivebase. */
  default void stop() {
    driveTank(0.0, 0.0);
  }
}
