// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.util.DeadBandGuard;
import frc.robot.util.PowerFunction;
import frc.robot.util.SpeedScaler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  private final Joystick m_driveJoystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  private final SpeedScaler m_speedScaler = new SpeedScaler(new SpeedScaler.ModeFunction() {
    PowerFunction turtleTrigger = new DeadBandGuard(() -> m_driveJoystick.getRawAxis(Constants.GAMESIR_LEFT_TRIGGER),
        0.25);
    PowerFunction turboTrigger = new DeadBandGuard(() -> m_driveJoystick.getRawAxis(Constants.GAMESIR_RIGHT_TRIGGER),
        0.25);

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
  }, 0.75 /* normal */, 1.0 /* turbo */, 0.5 /* turtle */);

  private final Command tankDriveCommand = generateTankDriveCommand();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_romiDrivetrain.setDefaultCommand(tankDriveCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  private TankDriveCommand generateTankDriveCommand() {
    // Raw inputs from the joysticks
    PowerFunction rawLeftPower = () -> -1 * m_driveJoystick.getRawAxis(Constants.GAMESIR_LEFT_VERTICAL);
    PowerFunction rawRightPower = () -> -1 * m_driveJoystick.getRawAxis(Constants.GAMESIR_RIGHT_VERTICAL);

    // Apply deadband protection
    PowerFunction guardedLeftPower = new DeadBandGuard(rawLeftPower, 0.06);
    PowerFunction guardedRightPower = new DeadBandGuard(rawRightPower, 0.06);

    // Apply speed scaler
    PowerFunction scaledLeftPower = m_speedScaler.apply(guardedLeftPower);
    PowerFunction scaledRightPower = m_speedScaler.apply(guardedRightPower);

    return new TankDriveCommand(m_romiDrivetrain, scaledLeftPower, scaledRightPower);
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
