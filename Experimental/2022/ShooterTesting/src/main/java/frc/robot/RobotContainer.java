// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShooterMaxPowerForward;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();

  private final Command autoCommand = new PrintCommand("Do something autonomously....");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("100% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 1.0));
    SmartDashboard.putData("90% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.90));
    SmartDashboard.putData("80% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.80));
    SmartDashboard.putData("70% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.70));
    SmartDashboard.putData("60% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.60));
    SmartDashboard.putData("50% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.50));
    SmartDashboard.putData("40% speed", new ShooterSetVelocity(shooter, Shooter.MAX_RPM * 0.40));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
