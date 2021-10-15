// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RainbowLightingCommand;
import frc.robot.commands.SimpleLightingCommand;
import frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LightingSubsystem m_exampleSubsystem = new LightingSubsystem(Constants.LED_PWM_PORT, Constants.LED_STRIP_LENGTH);

  // Sample command to change the color during auto mode.
  private final Command m_autoCommand = new RainbowLightingCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Lights will default to green, unless set otherwise.
    m_exampleSubsystem
        .setDefaultCommand(new SimpleLightingCommand(m_exampleSubsystem, SimpleLightingCommand.Mode.Green));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Returns the command to be executed in autonomous mode.
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
