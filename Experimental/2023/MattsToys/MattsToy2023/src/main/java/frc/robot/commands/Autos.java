// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto() {
    return Commands.sequence(
        new PrintCommand("Do something"),
        new PrintCommand("Do something else"));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
