// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.constants.LogitechConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;

public class RobotContainer {
  private final IDrivebase drivebase = new Drivebase();
  private final Joystick m_driveController = new Joystick(OperatorConstants.DRIVER_JOYSTICK_ID);
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  /** Constructor. */
  public RobotContainer() {
    drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(drivebase, this::getArcadeForward, this::getArcadeRotation));

    configureBindings();
  }

  private void configureBindings() {
    setupDriveControlSelection();

    // Set up autonomous command chooser
    autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    SmartDashboard.putData("Autonomous Command", autoCommandChooser);
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (autoCommandChooser.getSelected() != null) {
      return autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }

  //
  // Drivebase control support
  //

  /** Enumeration of available drive control schemes. */
  private enum DriveControl {
    KEYBOARD1,
    ALT_KEYBOARD1,
    LOGITECH_CONTROLLER,
    GAMESIR_CONTROLLER;

    /** Returns the name of the control scheme. */
    public String getControlSchemeName() {
      return switch (this) {
        case KEYBOARD1 -> "Keyboard1";
        case ALT_KEYBOARD1 -> "Alt-Keyboard1";
        case LOGITECH_CONTROLLER -> "Logitech Controller";
        case GAMESIR_CONTROLLER -> "GameSir Controller";
      };
    }
  }

  /** The currently selected drive control scheme. */
  DriveControl currentControlScheme = DriveControl.KEYBOARD1;

  /** Sets up the drive control selection on the SmartDashboard. */
  private void setupDriveControlSelection() {
    // Load the last-selected control scheme from preferences
    int savedControlSchemeOrdinal = Preferences.getInt("DriveControlScheme", 0);
    if (savedControlSchemeOrdinal < 0 || savedControlSchemeOrdinal >= DriveControl.values().length) {
      savedControlSchemeOrdinal = 0;
    }
    currentControlScheme = DriveControl.values()[savedControlSchemeOrdinal];

    // Build/install the chooser, establishing the saved scheme as the default
    SendableChooser<DriveControl> driveInputChooser = new SendableChooser<DriveControl>();
    for (var option : DriveControl.values()) {
      if (option == currentControlScheme) {
        driveInputChooser.setDefaultOption(option.getControlSchemeName(), option);
      } else {
        driveInputChooser.addOption(option.getControlSchemeName(), option);
      }
    }
    SmartDashboard.putData("Drive control", driveInputChooser);

    // Register a listener to update the control scheme when the user makes a
    // selection
    driveInputChooser.onChange(this::updateControlScheme);
  }

  /** Updates the current control scheme based on user selection. */
  private void updateControlScheme(DriveControl controlScheme) {
    currentControlScheme = controlScheme;
    Preferences.setInt("DriveControlScheme", controlScheme.ordinal());
    System.out.println("Control scheme set to: " + controlScheme.getControlSchemeName());
  }

  /** Slew rate limiter for forward control. */
  private final SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);

  /**
   * Returns the "forward" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeForward() {
    final double forward = switch (currentControlScheme) {
      case KEYBOARD1 -> m_driveController.getRawAxis(0); // Mapped to D/A keys
      case ALT_KEYBOARD1 -> -m_driveController.getRawAxis(1); // Mapped to W/S keys
      case LOGITECH_CONTROLLER ->
        -m_driveController.getRawAxis(LogitechConstants.Dualshock.LeftYAxis);
      case GAMESIR_CONTROLLER ->
        -m_driveController.getRawAxis(frc.robot.constants.GameSirConstants.Axes.LEFT_Y);
    };
    return forwardSlewRateLimiter.calculate(
        MathUtil.applyDeadband(forward, OperatorConstants.DEADBAND_THRESHOLD));
  }

  /** Slew rate limiter for rotation control. */
  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);

  /**
   * Returns the "rotation" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeRotation() {
    final double rotation = switch (currentControlScheme) {
      case KEYBOARD1 -> -m_driveController.getRawAxis(1); // Mapped to W/S keys
      case ALT_KEYBOARD1 -> m_driveController.getRawAxis(0); // Mapped to D/A keys
      case LOGITECH_CONTROLLER ->
        -m_driveController.getRawAxis(LogitechConstants.Dualshock.RightXAxis);
      case GAMESIR_CONTROLLER ->
        -m_driveController.getRawAxis(frc.robot.constants.GameSirConstants.Axes.RIGHT_X);
    };
    return rotationSlewRateLimiter.calculate(
        MathUtil.applyDeadband(rotation, OperatorConstants.DEADBAND_THRESHOLD));
  }
}