// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.util.DeadBandGuard;
import frc.robot.util.PowerFunction;
import frc.robot.util.SpeedScaler;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Drive train subsystem. */
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  /** Joystick used for driving. */
  private final Joystick m_driveJoystick = new Joystick(DRIVER_JOYSTICK_PORT);

  /** Tank drive command, used to handle movement during teleop mode. */
  private final Command tankDriveCommand = generateTankDriveCommand();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_romiDrivetrain.setDefaultCommand(tankDriveCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Returns a TankDriveCommand, with all desired "dead band" guards, and support
   * for speed scaling (turbo/turtle modes).
   * 
   * @see #m_driveJoystick
   */
  private TankDriveCommand generateTankDriveCommand() {
    // Raw inputs from the joysticks
    PowerFunction rawLeftPower = () -> -1 * m_driveJoystick.getRawAxis(GAMESIR_LEFT_VERTICAL);
    PowerFunction rawRightPower = () -> -1 * m_driveJoystick.getRawAxis(GAMESIR_RIGHT_VERTICAL);

    // Apply deadband protection
    PowerFunction guardedLeftPower = new DeadBandGuard(rawLeftPower, THROTTLE_DEADBAND_LIMIT);
    PowerFunction guardedRightPower = new DeadBandGuard(rawRightPower, THROTTLE_DEADBAND_LIMIT);

    // Apply speed scaler
    SpeedScaler scaler = generateSpeedScaler();
    PowerFunction scaledLeftPower = scaler.apply(guardedLeftPower);
    PowerFunction scaledRightPower = scaler.apply(guardedRightPower);

    return new TankDriveCommand(m_romiDrivetrain, scaledLeftPower, scaledRightPower);
  }

  /**
   * Returns a SpeedScaler for use with driving the robot.
   * 
   * @see #m_driveJoystick
   * @see #generateTankDriveCommand()
   */
  private SpeedScaler generateSpeedScaler() {
    // Raw input from the triggers on the drive controller.
    PowerFunction rawTurtleTrigger = () -> m_driveJoystick.getRawAxis(GAMESIR_LEFT_TRIGGER);
    PowerFunction rawTurboTrigger = () -> m_driveJoystick.getRawAxis(GAMESIR_RIGHT_TRIGGER);

    // Deadband-guarded inputs, so that we're not subject to fluctuations in the
    // triggers.
    PowerFunction turtleTrigger = new DeadBandGuard(rawTurtleTrigger, SPEED_SCALING_DEADBAND_LIMIT);
    PowerFunction turboTrigger = new DeadBandGuard(rawTurboTrigger, SPEED_SCALING_DEADBAND_LIMIT);

    // Speed scaler.
    return new SpeedScaler(new SpeedScaler.ModeFunction() {
      @Override
      public SpeedScaler.Mode get() {
        if (turtleTrigger.get() > 0) {
          return SpeedScaler.Mode.TURTLE;
        } else if (turboTrigger.get() > 0) {
          return SpeedScaler.Mode.TURBO;
        } else {
          return SpeedScaler.Mode.NORMAL;
        }
      }
    }, NORMAL_MODE_SPEED_LIMT, TURBO_MODE_SPEED_LIMT, TURTLE_MODE_SPEED_LIMT);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
