// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.simulations.SimulatedSingleJointArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmWaveCommand extends Command {
  private final SimulatedSingleJointArm arm;
  private int counter;
  /**
   * How fast the command should spend on a full transition from one extreme to
   * the other and back.
   */
  private final int CYCLE_TIME_IN_SECONDS = 4;
  private final int CYCLE_TIME_IN_ITERATIONS = (CYCLE_TIME_IN_SECONDS * 50);

  /** Creates a new ArmWaveCommand. */
  public ArmWaveCommand(SimulatedSingleJointArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  static final boolean NOISY = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter % CYCLE_TIME_IN_ITERATIONS == CYCLE_TIME_IN_ITERATIONS / 2) {
      if (NOISY) {
        System.out.println("Setting to upright");
      }
      arm.setTargetPositionInRadians(Math.toRadians(90));
    } else if (counter % CYCLE_TIME_IN_ITERATIONS == 0) {
      if (NOISY) {
        System.out.println("Setting to flat");
      }
      arm.setTargetPositionInRadians(Math.toRadians(180));
    }

    // Increment counter for next pass
    ++counter;
  }
}
