// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LogitechGamePad;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ColorLights;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunOnlyConveyorMotor;
import frc.robot.commands.RunOnlyConveyorMotorReverse;
import frc.robot.commands.RunShootingMotor;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivebase m_drivebase = new Drivebase();
  private final Lights m_lights = new Lights();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();

  private final Joystick m_driverController = new Joystick(DRIVER_JOYSTICK_ID);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OPERATOR_JOYSTICK_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase,
        // Forward speed supplier
        ()
            -> m_driverController.getRawAxis(LogitechGamePad.LeftYAxis)
            * getDriveSpeedScalingFactor(),
        // Turn speed supplier
        ()
            -> m_driverController.getRawAxis(LogitechGamePad.RightXAxis)
            * getDriveSpeedScalingFactor()));

    m_lights.setDefaultCommand(new ColorLights(m_lights, Lights.GREEN));
  }

  /**
   * @return the scaling factor that should be applied to speed values read from
   *     the driver's joysticks (for normal, turbo, and turtle modes)
   */
  private double getDriveSpeedScalingFactor() {
    final boolean isTurbo =
        m_driverController.getRawButton(Constants.LogitechGamePad.RightShoulder);
    final boolean isTurtle =
        m_driverController.getRawButton(Constants.LogitechGamePad.LeftShoulder);

    if (isTurbo) {
      return Constants.RobotSpeedScaling.TURBO_MODE_SPEED_SCALING;
    } else if (isTurtle) {
      return Constants.RobotSpeedScaling.TURTLE_MODE_SPEED_SCALING;
    } else {
      return Constants.RobotSpeedScaling.NORMAL_MODE_SPEED_SCALING;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .whileTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_operatorController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    new Trigger(
        // Call this function (static or a lambda) to decide if we should
        // trigger something or not.
        () -> m_driverController.getRawButton(LogitechGamePad.XButton))
        // While the function called above returns true, run the specified command.
        .whileTrue(
            // Comand to be run while true
            new RunShootingMotor(m_shooter, .5));

    new Trigger(() -> m_driverController.getRawButton(LogitechGamePad.YButton))
        .whileTrue(new RunShootingMotor(m_shooter, 0.8));
    new Trigger(() -> m_driverController.getRawButton(LogitechGamePad.AButton))
        .whileTrue(new RunOnlyConveyorMotorReverse(m_intake));
    new Trigger(() -> m_driverController.getRawButton(LogitechGamePad.BButton))
        .whileTrue(new RunOnlyConveyorMotor(m_intake));
    // new Trigger(() ->
    // m_driverController.getRawButton(LogitechGamePad.LeftShoulder))
    // .onTrue(new InstantCommand(() -> m_shooter.SetServoPosition(1.0)));
    // new Trigger(() ->
    // m_driverController.getRawButton(LogitechGamePad.RightShoulder))
    // .onTrue(new InstantCommand(() -> m_shooter.SetServoPosition(0.0)));
  }

  boolean getABooleanResult() {
    return true;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
