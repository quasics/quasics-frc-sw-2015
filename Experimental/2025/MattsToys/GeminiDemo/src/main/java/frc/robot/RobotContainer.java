// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.RunManipulator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot "logic" should
 * actually reside in the {@link Robot} class. Instead, the {@link
 * RobotContainer} class should be used to instantiate subsystems, declare
 * commands, and bind button triggers to commands.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Manipulator m_manipulator = new Manipulator();

  // The driver's joystick
  private final Joystick m_driverJoystick =
      new Joystick(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands for subsystems
    m_drivetrain.setDefaultCommand(new DriveWithJoysticks(
        m_drivetrain,
        ()
            -> - m_driverJoystick.getRawAxis(1), // Left joystick Y-axis
        ()
            -> - m_driverJoystick.getRawAxis(
                     5) // Right joystick Y-axis (assuming Xbox controller right
                        // stick Y)
        ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * configured to call commands by mounting them on the {@link
   * edu.wpi.first.wpilibj2.command.button.CommandXboxController} or {@link
   * edu.wpi.first.wpilibj2.command.button.CommandJoystick} class.
   */
  private void configureButtonBindings() {
    // Example: Bind a button to run the manipulator forward when pressed, stop
    // when released
    new JoystickButton(
        m_driverJoystick,
        6) // Example: Button 6 (Right Bumper on an Xbox Controller)
        .whileTrue(
            new RunManipulator(m_manipulator, 0.7)); // Run forward at 70% speed

    // Example: Bind another button to run the manipulator backward
    new JoystickButton(
        m_driverJoystick,
        5) // Example: Button 5 (Left Bumper on an Xbox Controller)
        .whileTrue(new RunManipulator(m_manipulator,
                                      -0.7)); // Run backward at 70% speed
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example autonomous command that drives forward for 2 seconds
    return new AutoDrive(m_drivetrain, 0.5,
                         2.0); // Drive forward at 50% speed for 2 seconds
  }
}
