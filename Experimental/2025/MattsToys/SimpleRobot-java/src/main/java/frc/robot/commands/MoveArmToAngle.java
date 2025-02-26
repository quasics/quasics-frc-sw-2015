// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ISingleJointArm;

/**
 * Command to move the arm to a specific angle.
 *
 * Note that this is a "set it and done" sort of command, since *all* the
 * command needs to do is to configure the target position for the elevator.
 * Once that's been done, the PID logic on the elevator (or a simulated version
 * of it, at least) should continue driving the elevator subsystem to the
 * specified position (unless something else changes that value).
 */
public class MoveArmToAngle extends Command {
  /** Arm being controlled. */
  final private ISingleJointArm m_arm;
  /** Target angle. */
  final private Angle m_angle;

  /**
   * Creates a new MoveArmToAngle.
   *
   * @param arm   arm being controlled
   * @param angle angle to which the arm should be moved
   */
  public MoveArmToAngle(ISingleJointArm arm, Angle angle) {
    this.m_arm = arm;
    this.m_angle = angle;

    addRequirements(arm.asSubsystem());
  }

  @Override
  public void initialize() {
    m_arm.setTargetPosition(m_angle);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
