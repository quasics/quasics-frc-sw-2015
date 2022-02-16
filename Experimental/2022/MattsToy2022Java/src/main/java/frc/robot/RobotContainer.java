// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BreathingLights;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SimpleLighting;
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

  private final RobotSettings m_robotSettings = loadSettingsOrDefaults();

  private final DriveBase m_driveBase;
  private final Lighting m_lighting;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Log the settings which we'll be using during operations.
    System.out.println("*** Running with robot configuration:\n" + m_robotSettings);

    // Allocate the joystick for the driver.
    Joystick driverStick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);

    //////////////////////////////////////////////////////////////
    // Set up the drive base.

    m_driveBase = new DriveBase(m_robotSettings);

    // Configure tank drive command (default for drive base).
    DeadBandEnforcer drivingDeadband = new DeadBandEnforcer(Constants.Deadbands.DRIVING);
    SpeedScaler normalSpeedScaler = new SpeedScaler(Constants.SpeedLimits.MAX_SPEED_NORMAL);
    SpeedScaler turtleSpeedScaler = new SpeedScaler(Constants.SpeedLimits.MAX_SPEED_TURTLE);
    SpeedScaler turboSpeedScaler = new SpeedScaler(Constants.SpeedLimits.MAX_SPEED_TURBO);
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

    TankDrive tankDrive = new TankDrive(m_driveBase,
        // Left side control
        () -> drivingDeadband.adjustSpeed(
            modeScaler.adjustSpeed(
                driverStick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS))),
        // Right side control
        () -> drivingDeadband.adjustSpeed(
            modeScaler.adjustSpeed(
                driverStick.getRawAxis(LogitechGamePad.RIGHT_Y_AXIS))));
    m_driveBase.setDefaultCommand(tankDrive);

    //////////////////////////////////////////////////////////////
    // Set up the lighting subsystem.

    m_lighting = new Lighting(Constants.Lighting.PWM_PORT, Constants.Lighting.NUM_LIGHTS);
    m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));

    //////////////////////////////////////////////////////////////
    // Finish setting up commands on the stick(s) and dashboard.
    configureButtonBindings(driverStick);
    configureSmartDashboard();
  }

  private static final boolean LOAD_SETTINGS_FROM_DEPLOYED_FILES = false;
  private static final String DEFAULT_ROBOT_PROPERTY_NAME = "sally";

  // Only used iff LOAD_FROM_DEPLOYED_FILES == false.
  private static final String SETTINGS_FILE_NAME = "robotSettings.props";

  /**
   * Returns robot-specific settings from the "save file" (or else the defaults,
   * on errors).
   */
  private static RobotSettings loadSettingsOrDefaults() {
    RobotSettings settings = null;
    if (LOAD_SETTINGS_FROM_DEPLOYED_FILES) {
      // Add "robot properties" control field to dashboard (and make it persistent).
      SmartDashboard.setDefaultString("Properties", DEFAULT_ROBOT_PROPERTY_NAME);
      SmartDashboard.setPersistent("Properties");

      // Get the robot properties from the deployed file for the named robot.
      final String robotFileName = SmartDashboard.getString("Properties", DEFAULT_ROBOT_PROPERTY_NAME) + ".props";
      settings = RobotSettings.loadFromDeployedFile(robotFileName);
      if (settings != null) {
        return settings;
      }

      System.err.println(
          "---------------------------------------------------------------------------\n"
              + "Couldn't load robot settings from deployed " + robotFileName + ": falling back on defaults!\n"
              + "\n"
              + "Please update robot properties selection on the dashboard (e.g., 'sally').\n"
              + "---------------------------------------------------------------------------\n");
      SmartDashboard.putString("Properties", DEFAULT_ROBOT_PROPERTY_NAME);
    } else {
      settings = RobotSettings.loadFromFile(SETTINGS_FILE_NAME);
      if (settings != null) {
        return settings;
      }

      System.err.println(
          "------------------------------------------------------------\n"
              + "Couldn't load robot settings: falling back on defaults!\n"
              + "\n"
              + "Please write current settings out to file via dashboard.\n"
              + "------------------------------------------------------------\n");
    }

    // Return the default property set.
    return getDefaultSettings();
  }

  private static final double INCHES_PER_METER = 39.3701;

  private static RobotSettings getSettingsForSally() {
    return new RobotSettings(
        "Sally", // robotName
        Constants.TRACK_WIDTH_INCHES_SALLY / INCHES_PER_METER,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  private static RobotSettings getSettingsForMae() {
    return new RobotSettings(
        "Mae", // robotName
        Constants.TRACK_WIDTH_INCHES_MAE / INCHES_PER_METER,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  private static RobotSettings getSettingsForNike() {
    return new RobotSettings(
        "Nike", // robotName
        Constants.TRACK_WIDTH_INCHES_NIKE / INCHES_PER_METER,
        true, // leftMotorsInverted
        false // RIGHT_MOTORS_INVERTED_PROPERTY
    );
  }

  private static RobotSettings getDefaultSettings() {
    return getSettingsForSally();
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

  private static void writeSettingsToFile(RobotSettings settings) {
    if (settings.writeToFile(SETTINGS_FILE_NAME)) {
      System.out.println("Saved settings for " + settings.robotName);
    } else {
      System.err.println("**** Failed to save settings for " + settings.robotName);
    }
  }

  private void configureSmartDashboard() {
    // Buttons to allow writing settings files (for use on next boot).
    //
    // TODO(mjh): If this works, switch back to using buttons to swap stored
    // settings, and refresh on reload.
    SmartDashboard.putData("Store Sally", new InstantCommand(() -> {
      writeSettingsToFile(getDefaultSettings());
    }));
    SmartDashboard.putData("Store Mae", new InstantCommand(() -> {
      writeSettingsToFile(getSettingsForMae());
    }));
    SmartDashboard.putData("Store Nike", new InstantCommand(() -> {
      writeSettingsToFile(getSettingsForNike());
    }));

    // Lighting commands
    SmartDashboard.putData("Red", new SimpleLighting(m_lighting, Lighting.StockColor.Red));
    SmartDashboard.putData("Blue", new SimpleLighting(m_lighting, Lighting.StockColor.Blue));
    SmartDashboard.putData("Green", new SimpleLighting(m_lighting, Lighting.StockColor.Green));
    SmartDashboard.putData("Breathe", new BreathingLights(m_lighting, Lighting.StockColor.Green));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Do something autonomously.... :-");
  }
}
