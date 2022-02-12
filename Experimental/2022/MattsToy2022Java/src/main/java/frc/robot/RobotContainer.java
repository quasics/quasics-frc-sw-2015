// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private static final String SETTINGS_FILE_NAME = "robotSettings.props";

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

  /**
   * Returns robot-specific settings from the "save file" (or else the defaults,
   * on errors).
   */
  private static RobotSettings loadSettingsOrDefaults() {
    RobotSettings settings = RobotSettings.loadFromFile(SETTINGS_FILE_NAME);
    if (settings == null) {
      System.err.println(
          "------------------------------------------------------------\n"
              + "Couldn't load robot settings: falling back on defaults!\n"
              + "\n"
              + "Please write current settings out to file via dashboard.\n"
              + "------------------------------------------------------------\n");
      settings = getSettingsForSally();
    }
    return settings;
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

    // Lighting commands
    SmartDashboard.putData("Red", new SimpleLighting(m_lighting, Lighting.Color.Red));
    SmartDashboard.putData("Blue", new SimpleLighting(m_lighting, Lighting.Color.Blue));
    SmartDashboard.putData("Green", new SimpleLighting(m_lighting, Lighting.Color.Green));
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
