// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  final private ISingleJointArm arm;
  final private double angleRadians;

  /** Creates a new MoveArmToAngle. */
  public MoveArmToAngle(ISingleJointArm arm, double angleRadians) {
    this.arm = arm;
    this.angleRadians = angleRadians;

    addRequirements(arm.asSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTargetPositionInRadians(angleRadians);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
