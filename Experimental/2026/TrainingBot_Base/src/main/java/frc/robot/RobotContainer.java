// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.RealNovaDrivebase;
import frc.robot.subsystems.SimulatedDrivebase;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared.
 * 
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////////
  // The following variables are used to control how the robot is configured when
  // running in simulation vs. on the real robot.

  // Establish how we'll control the robot's movement under simulation.
  private static final boolean USE_KEYBOARD_DRIVING_UNDER_SIMULATION = true;

  private static final boolean USING_REAL_CONTROLLER =
      // Assume we have a real controller available if either:
      // 1) The robot is real (so we're using an actual driver station + joystick).
      // 2) The robot isn't real, but we're *not* using the keyboard under simulation.
      Robot.isReal() || !USE_KEYBOARD_DRIVING_UNDER_SIMULATION;

  /////////////////////////////////////////////////////////////////////////////////
  // The robot's subsystems are defined here.
  //
  // For this simple example, we only have a single subsystem (the drivebase), and
  // a few commands to control it. In a more complex robot, you would likely have
  // multiple subsystems defined here.

  /**
   * The robot's drivebase, which supports all movement-related functionality,
   * including related sensors.
   */
  private final AbstractDrivebase m_driveBase;

  /////////////////////////////////////////////////////////////////////////////////
  // The "Human Interface Devices" (HIDs) are defined here. These are the devices
  // that the drivers will use to control the robot (e.g., joysticks, gamepads,
  // etc.).
  //
  // We also define "supplier" objects for the joystick axes that will be used for
  // driving, so that we can abstract away the specific controller/joystick
  // configuration from the commands that use these values, which is a good thing
  // since we want to be able to use the same commands in both simulation and on
  // the real robot, even if the specific controller/joystick configuration is
  // different in those two environments.
  //
  // This can also let us apply some transformations to the raw joystick values
  // (e.g., inverting them, applying a deadband, supporting "turbo" and "turtle"
  // modes, etc.) in a single place, without having to worry about that logic
  // being duplicated across multiple commands.

  /** The driver's controller. */
  private final CommandJoystick m_driverController = new CommandJoystick(0);

  /**
   * A "supplier" object which will be used by commands to access the left
   * joystick's Y-axis value.
   */
  private final Supplier<Double> m_leftSupplier;

  /**
   * A "supplier" object which will be used by commands to access the left
   * joystick's Y-axis value.
   */
  private final Supplier<Double> m_rightSupplier;

  /////////////////////////////////////////////////////////////////////////////////
  // The folowing are the functions supported by the RobotContainer class, which
  // are used to configure the robot's subsystems, commands, and trigger bindings.

  /**
   * Constructor.
   */
  public RobotContainer() {
    // Allocate our subsystems.
    if (Robot.isReal()) {
      // Configuring the real robot.
      m_driveBase = new RealNovaDrivebase();
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
      // axis 0 & 1 (from the "Keyboard 0" configuration).
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

    // Another example, using a separate function to create/return the command
    // (in this case, a command sequence) to be added behind a dashboard button.
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
   * Defines any trigger->command mappings.
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
   * Returns the command to be executed in autonomous mode.
   * 
   * This will be invoked by the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("Do something autonomously....");
  }
}
