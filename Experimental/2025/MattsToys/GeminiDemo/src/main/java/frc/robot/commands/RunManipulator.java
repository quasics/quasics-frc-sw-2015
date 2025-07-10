// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class RunManipulator extends Command {
  private final Manipulator m_manipulator;
  private final double m_speed;

  /**
   * Creates a new RunManipulator command.
   *
   * @param manipulator The manipulator subsystem this command will operate on.
   * @param speed The speed at which to run the manipulator (-1.0 to 1.0).
   */
  public RunManipulator(Manipulator manipulator, double speed) {
    m_manipulator = manipulator;
    m_speed = speed;
    addRequirements(m_manipulator); // Declare that this command requires the
                                    // Manipulator subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.setSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.stop(); // Stop the manipulator when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This command will run until interrupted (e.g., button
                  // released)
  }
}