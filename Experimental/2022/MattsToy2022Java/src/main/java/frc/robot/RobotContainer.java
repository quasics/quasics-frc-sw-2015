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
import frc.robot.Constants.OperatorInterface.LogitechGamePad;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private static final String SETTINGS_FILE_NAME = "Sally";

  private final DriveBase driveBase;
  private final Lighting lighting = new Lighting(Constants.Lighting.PWM_PORT, Constants.Lighting.NUM_LIGHTS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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

    driveBase = new DriveBase(settings);

    // Allocate the joystick for the driver.
    Joystick driverStick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);

    // Configure the button bindings.
    configureButtonBindings(driverStick);

    // Configure tank drive command.
    DeadBandEnforcer drivingDeadband = new DeadBandEnforcer(Constants.Deadbands.DRIVING);
    TankDrive tankDrive = new TankDrive(driveBase,
        () -> drivingDeadband.restrict(driverStick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS)),
        () -> drivingDeadband.restrict(driverStick.getRawAxis(LogitechGamePad.RIGHT_Y_AXIS)));
    driveBase.setDefaultCommand(tankDrive);

    // Configure default lighting command.
    lighting.setDefaultCommand(new RainbowLighting(lighting));

    //
    // Write out settings files (for use on next boot)
    SmartDashboard.putData("Setup for Sally", new InstantCommand(() -> {
      getSettingsForSally().writeToFile(SETTINGS_FILE_NAME);
    }));
    SmartDashboard.putData("Setup for Mae", new InstantCommand(() -> {
      getSettingsForMae().writeToFile(SETTINGS_FILE_NAME);
    }));
    SmartDashboard.putData("Setup for Nike", new InstantCommand(() -> {
      getSettingsForNike().writeToFile(SETTINGS_FILE_NAME);
    }));
  }

  private static RobotSettings getSettingsForSally() {
    return new RobotSettings("Sally", Constants.TRACK_WIDTH_INCHES_SALLY);
  }

  private static RobotSettings getSettingsForMae() {
    return new RobotSettings("Mae", Constants.TRACK_WIDTH_INCHES_MAE);
  }

  private static RobotSettings getSettingsForNike() {
    return new RobotSettings("Nike", Constants.TRACK_WIDTH_INCHES_NIKE);
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
