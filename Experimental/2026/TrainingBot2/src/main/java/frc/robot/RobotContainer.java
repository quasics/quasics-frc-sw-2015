// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LogitechGamePad;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnRobot;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.RealDrivebase;
import frc.robot.subsystems.SimulatedDrivebase;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AbstractDrivebase m_driveBase;

  // Establish how we'll control the robot's movement under simulation.
  private static final boolean USE_KEYBOARD_DRIVING_UNDER_SIMULATION = true;

  private static final boolean USING_REAL_CONTROLLER =
      // Assume we have a real controller available if either:
      // 1) The robot is real (so we're using an actual driver station + joystick).
      // 2) The robot isn't real, but we're *not* using the keyboard under simulation.
      Robot.isReal() || !USE_KEYBOARD_DRIVING_UNDER_SIMULATION;

  private final CommandJoystick m_driverController = new CommandJoystick(0);

  private final Supplier<Double> m_leftSupplier;
  private final Supplier<Double> m_rightSupplier;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Real vs. Simulation-specific configuration
    if (Robot.isReal()) {
      // Configuring the real robot.
      m_driveBase = new RealDrivebase();
    } else {
      // Configuring the simulated robot
      m_driveBase = new SimulatedDrivebase();
    }

    // Set up the driving controls.
    if (USING_REAL_CONTROLLER) {
      // Note that we're inverting the values because Xbox and similar controllers
      // return negative values when we push forward.
      m_leftSupplier = () -> -m_driverController.getRawAxis(LogitechGamePad.LeftYAxis);
      m_rightSupplier = () -> -m_driverController.getRawAxis(LogitechGamePad.RightYAxis);
    } else {
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      m_leftSupplier = () -> m_driverController.getRawAxis(0);
      m_rightSupplier = () -> m_driverController.getRawAxis(1);
    }

    // Set default commands for subsystems (using the driving controls).
    Command driveCommand = new TankDrive(m_driveBase, m_leftSupplier, m_rightSupplier);
    m_driveBase.setDefaultCommand(driveCommand);

    // Add other commands to the dashboard.
    addCommandsToDashboard();

    // Configure the trigger bindings (if any).
    configureBindings();
  }

  /**
   * Add some sample commands to the dashboard for testing purposes.
   */
  private void addCommandsToDashboard() {
    // Examples of putting a button on the SmartDashboard to run a command.
    SmartDashboard.putData("3 feet, 25%",
        new DriveForDistance(m_driveBase, 0.25, Feet.of(3)));
    SmartDashboard.putData("3 meters, 50%",
        new DriveForDistance(m_driveBase, 0.5, Meters.of(3)));

    SmartDashboard.putData("1m @ 10%", new DriveForDistance(m_driveBase, 0.10, Meters.of(1)));
    SmartDashboard.putData("-1m @ 10%", new DriveForDistance(m_driveBase, 0.10, Meters.of(-1)));

    SmartDashboard.putData("Turn 90 @ 50%",
        new TurnRobot(m_driveBase, 0.5, Degrees.of(90)));

    // Another example, using a separate function to create/return the command
    // (sequence) to be added behind a dashboard button.
    SmartDashboard.putData("Move forward then backward", moveForwardThenBackward(m_driveBase));
  }

  /**
   * Example of building a sequence of commands to be executed (a form of
   * "command composition", as the WPILib docs call them).
   * 
   * There's also options for *parallel* execution of commands (i.e., do these N
   * different things at the same time), but since we've only really got one
   * subsystem to work with, and commands take exclusive control of the subsystems
   * that they use while they're running, there's not a lot we can do with
   * those....
   * 
   * @see https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.htm
   */
  public static Command moveForwardThenBackward(AbstractDrivebase drivebase) {
    return Commands.sequence(
        new DriveForDistance(drivebase, 0.30, 1), new DriveForDistance(drivebase, 0.30, -1));
  }

  /**
   * Use this method to define your trigger->command mappings.
   *
   * Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox} and
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers, or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (USING_REAL_CONTROLLER) {
      // Run a command (to print something) when someone pushes button #1 on the
      // controller.
      new Trigger(() -> m_driverController.getHID().getRawButton(1))
          .onTrue(new PrintCommand("Driver button 1 pressed"));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("Do something autonomously....");
  }
}
