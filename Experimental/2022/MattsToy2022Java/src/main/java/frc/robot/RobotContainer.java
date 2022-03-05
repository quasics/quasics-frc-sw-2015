// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BreathingLights;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SimpleLighting;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.AbstractDriveBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.subsystems.RomiDriveBase;
import frc.robot.utils.DeadBandEnforcer;
import frc.robot.utils.DrivePowerSupplier;
import frc.robot.utils.PropsIO;
import frc.robot.utils.SpeedScaler;
import frc.robot.utils.SwitchDriveHandler;
import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;
import frc.robot.utils.TurboTurtleScaler;
import frc.robot.Constants.OperatorInterface.GameSirPro;
import frc.robot.Constants.OperatorInterface.LogitechGamePad;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * If true, software will automatically configure the robot for use on a Romi.
   * 
   * Note: this is based on the detection of a file named ".simulatingRomi" in the
   * operating directory.
   */
  private final static boolean CONFIGURE_FOR_ROMI = new File(Filesystem.getOperatingDirectory(), ".simulatingRomi")
      .exists();

  /**
   * Controls whether robot settings/configuration data will be loaded from
   * (read-only) files in the "deploy" directory on the robot (iff == true), or
   * if they will be written to/loaded from a single file in the operating
   * directory (iff == false).
   * 
   * In either case, changes to the configured robot settings will only be applied
   * when the robot code is restarted.
   * 
   * However, the "deployed files" option has the advantage of allowing the robot
   * configuration to be changed solely by updating the choice in the UI and
   * restarting the robot code; the "store the config to be used" option requires
   * that the robot be *enabled* first, in order to allow the command to be
   * executed (which could be more of an issue at an event).
   * 
   * TODO(mjh): Consider updating the code to allow dynamic reloads, though this
   * would potentially imply regenerating one or more of the subsystems (thus they
   * couldn't be "final"), as well as a potential loss of some state if it happens
   * after some period of operations (e.g., odometry data).
   */
  private static final boolean LOAD_SETTINGS_FROM_DEPLOYED_FILES = false;

  /**
   * Default robot selection for loading robot settings/configuration data from
   * deployed property files.
   * 
   * Only used iff LOAD_FROM_DEPLOYED_FILES == true.
   * 
   * @see #loadSettingsOrDefaults()
   * @see frc.robot.RobotSettings#loadFromDeployedFile(String)
   */
  private static final String DEFAULT_ROBOT_PROPERTY_NAME = "sally";

  /**
   * The file to be used to store the robot settings to be made active on the next
   * restart of the robot code.
   * 
   * Only used iff LOAD_FROM_DEPLOYED_FILES == false.
   * 
   * @see #loadSettingsOrDefaults()
   * @see frc.robot.RobotSettings#writeToFile(String)
   */
  private static final String SETTINGS_FILE_NAME = "robotSettings.props";

  /** Settings to use in configuring the robot. */
  private final RobotSettings m_robotSettings = loadSettingsOrDefaults();

  /** Drive base subsystem. */
  private final AbstractDriveBase m_driveBase;
  /** Lighting subsystem (unused on Romi). */
  private final Lighting m_lighting;
  /** OnBoardIO subsystem (only used on Romi). */
  private final OnBoardIO m_onboardIO;

  /**
   * The container for the robot. Contains (and connects) subsystems, OI devices,
   * and commands.
   */
  public RobotContainer() {
    System.out.println("Operating directory: " + Filesystem.getOperatingDirectory());

    // Log the settings which we'll be using during operations.
    System.out.println(
        "-----------------------------------------\n"
            + "*** Running with robot configuration:\n"
            + m_robotSettings + "\n"
            + "-----------------------------------------\n");

    // Allocate the joystick for the driver.
    Joystick driverStick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);

    //////////////////////////////////////////////////////////////////
    // Set up the drive base.

    if (!CONFIGURE_FOR_ROMI) {
      m_driveBase = new DriveBase(m_robotSettings);
      m_onboardIO = null;
    } else {
      m_driveBase = new RomiDriveBase(m_robotSettings);
      m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
    }

    SwitchDriveHandler switchDriveHandler = null;

    if (m_driveBase != null) {
      final int turtleModeButtonIndex = CONFIGURE_FOR_ROMI
          ? GameSirPro.LEFT_SHOULDER_BUTTON
          : LogitechGamePad.LEFT_TRIGGER;
      final int turboModeButtonIndex = CONFIGURE_FOR_ROMI
          ? GameSirPro.RIGHT_SHOULDER_BUTTON
          : LogitechGamePad.RIGHT_TRIGGER;
      final int leftStickIndex = (CONFIGURE_FOR_ROMI
          ? GameSirPro.LEFT_Y_AXIS
          : LogitechGamePad.LEFT_Y_AXIS);
      final int rightStickIndex = (CONFIGURE_FOR_ROMI
          ? GameSirPro.RIGHT_Y_AXIS
          : LogitechGamePad.RIGHT_Y_AXIS);

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
            return driverStick.getRawButton(turtleModeButtonIndex);
          },
          () -> { // Turbo mode signal
            return driverStick.getRawButton(turboModeButtonIndex);
          });
      DrivePowerSupplier leftStick = () -> drivingDeadband.adjustSpeed(
          modeScaler.adjustSpeed(
              driverStick.getRawAxis(leftStickIndex)));
      DrivePowerSupplier rightStick = () -> drivingDeadband.adjustSpeed(
          modeScaler.adjustSpeed(
              driverStick.getRawAxis(rightStickIndex)));

      // Need to hang onto this to allow reference from configureButtonBindings()
      // (though I could make it local if I just bound it here...).
      switchDriveHandler = new SwitchDriveHandler(leftStick, rightStick);

      TankDrive tankDrive = new TankDrive(
          m_driveBase,
          // Left side control
          switchDriveHandler.getLeftSupplier(),
          // Right side control
          switchDriveHandler.getRightSupplier());
      m_driveBase.setDefaultCommand(tankDrive);
    }

    //////////////////////////////////////////////////////////////////
    // Example of how to use Romi on-board I/O features (if enabled).

    if (m_onboardIO != null) {
      Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
      onboardButtonA
          .whenActive(new PrintCommand("Button A Pressed"))
          .whenInactive(new PrintCommand("Button A Released"));
    }

    //////////////////////////////////////////////////////////////////
    // Set up the lighting subsystem.

    if (!CONFIGURE_FOR_ROMI) {
      m_lighting = new Lighting(Constants.Lighting.PWM_PORT, Constants.Lighting.NUM_LIGHTS);
      m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));
    } else {
      m_lighting = null;
    }

    //////////////////////////////////////////////////////////////////
    // Finish setting up commands on the stick(s) and dashboard.
    configureButtonBindings(driverStick, switchDriveHandler);
    configureSmartDashboard();
  }

  /**
   * Returns robot-specific settings from the "save file" (or else the defaults,
   * on errors).
   */
  private static RobotSettings loadSettingsOrDefaults() {
    if (CONFIGURE_FOR_ROMI) {
      return getSettingsForRomi();
    }

    RobotSettings settings = null;
    final String LOADING_ROBOT_PROPERTY_NAME = "Properties";
    if (LOAD_SETTINGS_FROM_DEPLOYED_FILES) {
      // Add "robot properties" control field to dashboard (and make it persistent).
      SmartDashboard.setDefaultString(LOADING_ROBOT_PROPERTY_NAME, DEFAULT_ROBOT_PROPERTY_NAME);
      SmartDashboard.setPersistent(LOADING_ROBOT_PROPERTY_NAME);

      // Get the robot properties from the deployed file for the named robot.
      final String specifiedRobotName = SmartDashboard
          .getString(LOADING_ROBOT_PROPERTY_NAME, DEFAULT_ROBOT_PROPERTY_NAME).trim();
      final String robotFileName = specifiedRobotName + ".props";
      try {
        settings = new RobotSettings(PropsIO.loadFromDeployedFile(robotFileName));
      } catch (IllegalArgumentException | IOException e) {
        System.err.println("Error loading settings from file '" + robotFileName + "'");
        e.printStackTrace();
      }
      if (settings != null) {
        return settings;
      }

      System.err.println(
          "---------------------------------------------------------------------------\n"
              + "Couldn't load robot settings from deployed " + robotFileName + ": falling back on defaults!\n"
              + "\n"
              + "Please update robot properties selection on the dashboard (e.g., 'sally').\n"
              + "---------------------------------------------------------------------------\n");
      SmartDashboard.putString(LOADING_ROBOT_PROPERTY_NAME, DEFAULT_ROBOT_PROPERTY_NAME);
    } else {
      // Remove the "load from deployed props" option from the dashboard, since it's
      // unused in the configured mode.
      SmartDashboard.clearPersistent(LOADING_ROBOT_PROPERTY_NAME);
      SmartDashboard.delete(LOADING_ROBOT_PROPERTY_NAME);

      // Try loading previously-saved robot settings from the file.
      try {
        settings = new RobotSettings(PropsIO.loadFromFile(SETTINGS_FILE_NAME));
      } catch (IllegalArgumentException | IOException e) {
        System.err.println("Error loading settings from file '" + SETTINGS_FILE_NAME + "'");
        e.printStackTrace();
      }
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

  /** Returns the robot settings for Sally (2022 robot). */
  private static RobotSettings getSettingsForSally() {
    return new RobotSettings(
        "Sally", // robotName
        Constants.TRACK_WIDTH_INCHES_SALLY / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_SALLY,
        // TODO(mjh) Calibrate Sally's values for kS, kV, and kA - these are from
        // 2021/Mae
        new DriveProfileData(0.31, 2.74, 0.249),
        // TODO(mjh) Calibrate Sally's values for PID control - these are from 2021/Mae
        new PIDConfig(2.28, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.ADXRS450,
        0 // pigeonCanID
    );
  }

  /** Returns the robot settings for Mae (2021 robot). */
  private static RobotSettings getSettingsForMae() {
    return new RobotSettings(
        "Mae", // robotName
        Constants.TRACK_WIDTH_INCHES_MAE / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_MAE,
        // Drive configuration constants (computed 01Mar2022 w/ SysId)
        new DriveProfileData(
            /* kS= */0.13895,
            /* kV= */1.3143,
            /* kA= */0.1935),
        // PID control constants (computed 01Mar2022 w/ SysId)
        new PIDConfig(0.0011379, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.ADXRS450,
        0 // pigeonCanID
    );
  }

  /** Returns the robot settings for Nike (2019 robot). */
  private static RobotSettings getSettingsForNike() {
    return new RobotSettings(
        "Nike", // robotName
        Constants.TRACK_WIDTH_INCHES_NIKE / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_NIKE,
        new DriveProfileData(
            /* kS= */0.14961,
            /* kV= */1.3717,
            /* kA= */0.1627),
        new PIDConfig(2.5682, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.Pigeon2,
        1 // pigeonCanID
    );
  }

  /** Returns the robot settings for use on a Romi. */
  private static RobotSettings getSettingsForRomi() {
    return new RobotSettings(
        "Romi", // robotName
        Constants.TRACK_WIDTH_METERS_ROMI,
        1, // TODO(mjh): Check gear ratio for the Romi
        // TODO(mjh): Recalibrate Romi's values for kS, kV, and kA (if SysId ever
        // supports this) - these are from 2021
        new DriveProfileData(1.25, 5.7, 0.0176),
        // TODO(mjh): Recalibrate Romi's values for PID control (if SysId ever
        // supports this) - these are from 2021
        new PIDConfig(0.00352, 0, 0),
        // Note: Romi docs indicate that it's the right motor that's inverted, but I run
        // the Romi in reverse because the USB camera is mounted on my upper deck that
        // way.
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.Romi,
        0 // pigeonCanID
    );
  }

  /** Returns default settings (for the 2022 FRC robot). */
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
   * @param driverStick        the driver's joystick.
   * @param switchDriveHandler tracks status for "switch drive".
   */
  private void configureButtonBindings(Joystick driverStick, SwitchDriveHandler switchDriveHandler) {
    final int switchDriveButtonIndex = CONFIGURE_FOR_ROMI
        ? GameSirPro.S_BUTTON
        : LogitechGamePad.START_BUTTON;

    JoystickButton b = new JoystickButton(driverStick, switchDriveButtonIndex);
    b.whenPressed(new InstantCommand(() -> {
      switchDriveHandler.switchDirections();
    }));
  }

  private static void writeSettingsToFile(RobotSettings settings) {
    try {
      PropsIO.writeToFile(settings, SETTINGS_FILE_NAME, "Configuration for " + settings.robotName);
      System.out.println("Saved settings for " + settings.robotName);
    } catch (IllegalArgumentException | IllegalAccessException | IOException e) {
      System.err.println("**** Failed to save settings for " + settings.robotName);
      e.printStackTrace();
    }
  }

  private void configureSmartDashboard() {
    // Buttons to allow writing settings files (for use on next boot).
    if (LOAD_SETTINGS_FROM_DEPLOYED_FILES) {
      SmartDashboard.delete("Store Sally");
      SmartDashboard.delete("Store Mae");
      SmartDashboard.delete("Store Nike");
    } else {
      SmartDashboard.putData("Store Sally", new InstantCommand(() -> {
        writeSettingsToFile(getSettingsForSally());
      }));
      SmartDashboard.putData("Store Mae", new InstantCommand(() -> {
        writeSettingsToFile(getSettingsForMae());
      }));
      SmartDashboard.putData("Store Nike", new InstantCommand(() -> {
        writeSettingsToFile(getSettingsForNike());
      }));
    }

    // Lighting commands
    if (m_lighting != null) {
      SmartDashboard.putData("Red", new SimpleLighting(m_lighting, Lighting.StockColor.Red));
      SmartDashboard.putData("Blue", new SimpleLighting(m_lighting, Lighting.StockColor.Blue));
      SmartDashboard.putData("Green", new SimpleLighting(m_lighting, Lighting.StockColor.Green));
      SmartDashboard.putData("Breathe", new BreathingLights(m_lighting, Lighting.StockColor.Green));
    }
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
