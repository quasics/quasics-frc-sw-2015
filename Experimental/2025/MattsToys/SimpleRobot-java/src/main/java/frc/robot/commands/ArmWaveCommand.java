// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ISingleJointArm;

/**
 * Sample command to wave the arm continuously between two positions.
 */
public class ArmWaveCommand extends Command {
  /** Arm being controlled. */
  private final ISingleJointArm arm;
  /** Used to control cycling between positions. */
  private int counter;

  /**
   * How fast the command should spend on a full transition from one extreme to
   * the other and back.
   */
  private final int CYCLE_TIME_IN_SECONDS = 4;

  /** Full cycle time in iterations (i.e., calls to "execute") at 50Hz. */
  private final int CYCLE_TIME_IN_ITERATIONS = (CYCLE_TIME_IN_SECONDS * 50);

  /**
   * Creates a new ArmWaveCommand.
   *
   * @param arm the arm being controlled
   */
  public ArmWaveCommand(ISingleJointArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm.asSubsystem());
  }

  @Override
  public void initialize() {
    counter = 0;
  }

  /** Iff true, report changes in arm's target position (for debugging). */
  static final boolean NOISY = false;

  @Override
  public void execute() {
    if (counter % CYCLE_TIME_IN_ITERATIONS == CYCLE_TIME_IN_ITERATIONS / 2) {
      if (NOISY) {
        System.out.println("Setting to upright");
      }
      arm.setTargetPosition(ISingleJointArm.ARM_UP_ANGLE);
    } else if (counter % CYCLE_TIME_IN_ITERATIONS == 0) {
      if (NOISY) {
        System.out.println("Setting to flat");
      }
      arm.setTargetPosition(ISingleJointArm.ARM_OUT_ANGLE);
    }

    // Increment counter for next pass
    ++counter;
  }
}
