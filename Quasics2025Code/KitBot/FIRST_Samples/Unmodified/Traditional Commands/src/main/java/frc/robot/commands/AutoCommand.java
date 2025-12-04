// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

// Command to run the robot at 1/2 power for 1 second in autonomous
public class AutoCommand extends Command {
  CANDriveSubsystem driveSubsystem;
  private Timer timer;
  private double seconds = 1.0;

  // Constructor. Runs only once when the command is first created.
  public AutoCommand(CANDriveSubsystem driveSubsystem) {
    // Save parameter for use later and initialize timer object.
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(driveSubsystem);
  }

  // Runs each time the command is scheduled. For this command, we handle starting
  // the timer.
  @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // drive at 1/2 speed
    driveSubsystem.driveArcade(0.5, 0.0);
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    driveSubsystem.driveArcade(0.0, 0.0);
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= seconds;
  }
}
