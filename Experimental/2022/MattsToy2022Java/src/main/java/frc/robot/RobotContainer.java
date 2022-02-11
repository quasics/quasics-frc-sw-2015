// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Lighting;
import frc.robot.utils.DeadBandEnforcer;
import frc.robot.utils.SpeedScaler;
import frc.robot.utils.TurboTurtleScaler;
import frc.robot.Constants.OperatorInterface.LogitechGamePad;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final String SETTINGS_FILE_NAME = "robotSettings.props";

  private final DriveBase driveBase;
  private final Lighting lighting = new Lighting(Constants.Lighting.PWM_PORT, Constants.Lighting.NUM_LIGHTS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Load robot-specific settings from file (or defaults).
    RobotSettings settings = RobotSettings.loadFromFile(SETTINGS_FILE_NAME);
    if (settings == null) {
      System.err.println(
          "------------------------------------------------------------\n"
              + "Couldn't load robot settings: falling back on defaults!\n"
              + "\n"
              + "Please write current settings out to file via dashboard."
              + "------------------------------------------------------------\n");
      settings = getSettingsForSally();
    }
    System.out.println("*** Running with robot configuration --> " + settings.robotName);

    // Finish allocating the subsystems that rely on settings data.
    driveBase = new DriveBase(settings);

    // Allocate the joystick for the driver.
    Joystick driverStick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);

    // Configure tank drive command (default for drive base).
    DeadBandEnforcer drivingDeadband = new DeadBandEnforcer(Constants.Deadbands.DRIVING);
    SpeedScaler normalSpeedScaler = new SpeedScaler(0.65); // Limits speed to 65% of max (normal)
    SpeedScaler turtleSpeedScaler = new SpeedScaler(0.50); // Limits speed to 50% of max (turtle)
    SpeedScaler turboSpeedScaler = new SpeedScaler(0.80); // Limits speed to 80% of max (turbo)
    TurboTurtleScaler modeScaler = new TurboTurtleScaler(
        normalSpeedScaler,
        turtleSpeedScaler,
        turboSpeedScaler,
        () -> { // Turtle mode signal
          return driverStick.getRawButton(Constants.OperatorInterface.LogitechGamePad.LEFT_TRIGGER);
        },
        () -> { // Turbo mode signal
          return driverStick.getRawButton(Constants.OperatorInterface.LogitechGamePad.RIGHT_TRIGGER);
        });

    TankDrive tankDrive = new TankDrive(driveBase,
        // Left side control
        () -> drivingDeadband.adjustSpeed(
            modeScaler.adjustSpeed(
                driverStick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS))),
        // Right side control
        () -> drivingDeadband.adjustSpeed(
            modeScaler.adjustSpeed(
                driverStick.getRawAxis(LogitechGamePad.RIGHT_Y_AXIS))));
    driveBase.setDefaultCommand(tankDrive);

    // Configure default lighting command.
    lighting.setDefaultCommand(new RainbowLighting(lighting));

    //////////////////////////////////////////////////////////////
    // Finish setting up commands on the stick(s) and dashboard.
    configureButtonBindings(driverStick);
    configureSmartDashboard();
  }

  private static RobotSettings getSettingsForSally() {
    return new RobotSettings(
        "Sally", // robotName
        Constants.TRACK_WIDTH_INCHES_SALLY,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  private static RobotSettings getSettingsForMae() {
    return new RobotSettings(
        "Mae", // robotName
        Constants.TRACK_WIDTH_INCHES_MAE,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  private static RobotSettings getSettingsForNike() {
    return new RobotSettings(
        "Nike", // robotName
        Constants.TRACK_WIDTH_INCHES_NIKE,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  /**
   * Use this method to define your button->command mappings.
   *
   * Note that buttons can be created by instantiating a {@link GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * @param driverStick the driver's joystick.
   */
  private void configureButtonBindings(Joystick driverStick) {
  }

  private void configureSmartDashboard() {
    // Buttons to allow updating settings files (for use on next boot)
    SmartDashboard.putData("Sally on restart", new InstantCommand(() -> {
      getSettingsForSally().writeToFile(SETTINGS_FILE_NAME);
    }));
    SmartDashboard.putData("Mae on restart", new InstantCommand(() -> {
      getSettingsForMae().writeToFile(SETTINGS_FILE_NAME);
    }));
    SmartDashboard.putData("Nike on restart", new InstantCommand(() -> {
      getSettingsForNike().writeToFile(SETTINGS_FILE_NAME);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Do something autonomous.... :-");
  }
}
