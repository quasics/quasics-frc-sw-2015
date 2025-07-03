// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveWithJoysticks extends Command {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_leftJoystick;
  private final DoubleSupplier m_rightJoystick;

  /**
   * Creates a new DriveWithJoysticks command.
   *
   * @param drivetrain The drivetrain subsystem this command will operate on.
   * @param leftJoystick A supplier for the left joystick's Y-axis value.
   * @param rightJoystick A supplier for the right joystick's Y-axis value.
   */
  public DriveWithJoysticks(Drivetrain drivetrain, DoubleSupplier leftJoystick,
                            DoubleSupplier rightJoystick) {
    m_drivetrain = drivetrain;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    addRequirements(m_drivetrain); // Declare that this command requires the
                                   // Drivetrain subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_leftJoystick.getAsDouble(),
                       m_rightJoystick.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop(); // Stop the motors when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This command should run indefinitely as a default command
  }
}
