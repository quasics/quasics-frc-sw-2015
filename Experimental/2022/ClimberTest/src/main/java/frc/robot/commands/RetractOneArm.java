// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractOneArm extends CommandBase {
  final Climber climber;
  final boolean leftSide;

  /** Creates a new RetractOneArm. */
  public RetractOneArm(Climber climber, boolean leftSide) {
    this.climber = climber;
    this.leftSide = leftSide;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.retractSingleArm(leftSide);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
