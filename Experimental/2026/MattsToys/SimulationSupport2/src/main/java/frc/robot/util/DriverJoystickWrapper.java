// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.LogitechConstants;

/**
 * Wrapper around the driver's joystick to provide different control schemes.
 *
 * This also includes the SmartDashboard integration to allow the user to select
 * the desired control scheme at runtime, and the ability to save/load the last
 * selected scheme to/from preferences.
 *
 * This makes it easy to switch between different control schemes without
 * changing code, and allows for quick testing of different schemes (or on
 * machines where an option may/may not be at hand) during development.
 *
 * Note:
 * <ul>
 * <li>The "Keyboard1" and "Alt-Keyboard1" schemes assume that system "keyboard
 * 0" and "keyboard 1" are mapped to "Joystick 0" and "Joystick 1"
 * (respectively), e.g., in the simulator. Keyboard 0 defaults to using "D/A"
 * and "W/S" as up/down pairs, while Keyboard 1 defaults to "I/J" and "K/L".
 * Buttons "X", "Y", "A", and "B" are mapped to Joystick 0 buttons 2 (defaulting
 * to "X"), 1 ("Z"), 3 ("C"), and 4 ("V").
 *
 * <li>In the "Alt-Keyboard1" scheme, the "tank*" and "arcade*" commands all
 * work from a single controller (Joystick 0), and should default to using "D/A"
 * for forward control and "W/S" for rotation. In the other modes, the "left"
 * and "right" sticks on a combined controller (or keyboard 0 and 1 in the
 * "Keyboard1" scheme) are used.
 *
 * <li>The "GameSir Pro" scheme is based on the GameSir Pro controller's default
 * axis mapping, and represents a Mac-compatible Bluetooth controller that I
 * have handy.
 *
 * <li>Values for the Logitech, GameSir, and XBox joystick readings are being
 * inverted (negated) from their raw values because these controllers return
 * negative values when we push forward.
 *
 * <li>In all control schemes, if DISABLED_IN_AUTONOMOUS is true, the joystick
 * inputs will be ignored (returning 0.0) when the robot is in autonomous mode.
 * </ul>
 */
public final class DriverJoystickWrapper {
  /**
   * Whether to disable the joystick (and simply return 0 values) in autonomous
   * mode.
   */
  private static final boolean DISABLED_IN_AUTONOMOUS = true;

  /** The preference key for saving/loading the drive control scheme. */
  private static final String PREFERENCE_KEY_DRIVE_CONTROL_SCHEME = "DriveControlScheme";

  /** Enumeration of available drive control schemes. */
  public enum ControllerType {
    /** Keyboard control scheme 1. */
    KEYBOARD1,
    /**
     * Alternate version of KEYBOARD1, swapping the axis pairs on joystick 0.
     */
    ALT_KEYBOARD1,
    /** Logitech DualShock controller. */
    LOGITECH_DUALSHOCK_CONTROLLER,
    /** Xbox controller. */
    XBOX_CONTROLLER,
    /** GameSir Pro controller. */
    GAMESIR_CONTROLLER;

    /** Returns the name of the control scheme. */
    public String getControlSchemeName() {
      return switch (this) {
        case KEYBOARD1 -> "Keyboard1";
        case ALT_KEYBOARD1 -> "Alt-Keyboard1";
        case LOGITECH_DUALSHOCK_CONTROLLER -> "Logitech DualShock";
        case XBOX_CONTROLLER -> "Xbox";
        case GAMESIR_CONTROLLER -> "GameSir Pro";
      };
    }
  }

  /** The underlying joystick being wrapped. */
  private final Joystick m_primaryController;

  /** A secondary joystick, only used in keyboard modes. */
  private final Joystick m_secondaryController;

  /** Whether to save/load the selected control scheme to/from preferences. */
  private final boolean m_saveToPreferences;

  /** The currently selected drive control scheme. */
  private ControllerType currentControlScheme = ControllerType.KEYBOARD1;

  /** The deadband threshold for joystick inputs. */
  private double m_deadbandThreshold = 0.05;

  /**
   * Constructor. (Also adds the drive control selection to the SmartDashboard.)
   *
   * @param joystickId the ID of the joystick to wrap
   */
  public DriverJoystickWrapper(int joystickId) {
    this(joystickId, true);
  }

  /**
   * Constructor. (Also adds the drive control selection to the SmartDashboard.)
   *
   * Note: in keyboard modes, the joystick with id (joystickId+1) will also be
   * used for some functionality.
   *
   * @param joystickId        the ID of the joystick to wrap
   * @param saveToPreferences whether to save/load the selected control scheme
   *                          to/from preferences
   */
  public DriverJoystickWrapper(int joystickId, boolean saveToPreferences) {
    m_primaryController = new Joystick(joystickId);
    m_secondaryController = new Joystick(joystickId + 1);
    m_saveToPreferences = saveToPreferences;

    if (Robot.isReal()) {
      // In real robot, default to the Logitech controller scheme
      currentControlScheme = ControllerType.LOGITECH_DUALSHOCK_CONTROLLER;
    } else if (m_saveToPreferences) {
      // Load the last-selected control scheme from preferences
      int savedControlSchemeOrdinal = m_saveToPreferences
          // Load the last-selected control scheme from preferences
          ? Preferences.getInt(PREFERENCE_KEY_DRIVE_CONTROL_SCHEME, 0)
          // Default to 0 if not working with preferences
          : 0;
      if (savedControlSchemeOrdinal < 0 ||
          savedControlSchemeOrdinal >= ControllerType.values().length) {
        savedControlSchemeOrdinal = 0;
      }
      currentControlScheme = ControllerType.values()[savedControlSchemeOrdinal];
    }
    System.out.println("Driving control scheme set to: " +
        currentControlScheme.getControlSchemeName());

    addDriveControlSelectionToSmartDashboard();
  }

  /** Returns the currently selected control scheme. */
  public ControllerType getCurrentControlScheme() {
    return currentControlScheme;
  }

  /**
   * Sets the deadband threshold for joystick inputs.
   *
   * @param threshold the deadband threshold to set
   */
  public void setDeadbandThreshold(double threshold) {
    m_deadbandThreshold = threshold;
  }

  /** Sets up the drive control selection on the SmartDashboard. */
  private void addDriveControlSelectionToSmartDashboard() {
    // Build/install the chooser, establishing the saved scheme as the default
    SendableChooser<ControllerType> driveInputChooser = new SendableChooser<ControllerType>();
    for (var option : ControllerType.values()) {
      if (option == currentControlScheme) {
        driveInputChooser.setDefaultOption(option.getControlSchemeName(),
            option);
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
  private void updateControlScheme(ControllerType controlScheme) {
    currentControlScheme = controlScheme;
    System.out.println("Driving control scheme set to: " +
        currentControlScheme.getControlSchemeName());

    if (m_saveToPreferences) {
      Preferences.setInt(PREFERENCE_KEY_DRIVE_CONTROL_SCHEME,
          controlScheme.ordinal());
    }
  }

  /**
   * Returns the "forward" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeForward() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double forward = switch (currentControlScheme) {
      case ALT_KEYBOARD1 ->
        -m_primaryController.getRawAxis(1); // Mapped to W/S keys
      case KEYBOARD1, LOGITECH_DUALSHOCK_CONTROLLER, XBOX_CONTROLLER,
          GAMESIR_CONTROLLER ->
        getLeftY();
    };
    return MathUtil.applyDeadband(forward, m_deadbandThreshold);
  }

  /**
   * Returns the "rotation" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeRotation() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double rotation = switch (currentControlScheme) {
      case ALT_KEYBOARD1 ->
        m_primaryController.getRawAxis(0); // Mapped to D/A keys
      case KEYBOARD1, LOGITECH_DUALSHOCK_CONTROLLER, XBOX_CONTROLLER,
          GAMESIR_CONTROLLER ->
        getRightX();
    };
    return MathUtil.applyDeadband(rotation, m_deadbandThreshold);
  }

  /**
   * Returns the "left" value for tank drive based on the current control
   * scheme.
   */
  public Double getTankLeft() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double left = switch (currentControlScheme) {
      case KEYBOARD1 -> m_primaryController.getRawAxis(0); // Mapped to D/A keys
      case ALT_KEYBOARD1 ->
        -m_primaryController.getRawAxis(1); // Mapped to W/S keys
      case LOGITECH_DUALSHOCK_CONTROLLER, XBOX_CONTROLLER, GAMESIR_CONTROLLER ->
        getLeftY();
    };
    return MathUtil.applyDeadband(left, m_deadbandThreshold);
  }

  /**
   * Returns the "right" value for tank drive based on the current control
   * scheme.
   */
  public Double getTankRight() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double right = switch (currentControlScheme) {
      case KEYBOARD1 ->
        -m_primaryController.getRawAxis(1); // Mapped to W/S keys
      case ALT_KEYBOARD1 ->
        m_primaryController.getRawAxis(0); // Mapped to D/A keys
      case LOGITECH_DUALSHOCK_CONTROLLER, XBOX_CONTROLLER, GAMESIR_CONTROLLER ->
        getRightY();
    };
    return MathUtil.applyDeadband(right, m_deadbandThreshold);
  }

  /** Returns the left X axis value based on the current control scheme. */
  public Double getLeftX() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double leftX = switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        -m_primaryController.getRawAxis(0); // Mapped to A/D keys;
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        -m_primaryController.getRawAxis(LogitechConstants.Dualshock.LeftXAxis);
      case GAMESIR_CONTROLLER ->
        -m_primaryController.getRawAxis(
            frc.robot.constants.GameSirConstants.Axes.LEFT_X);
      case XBOX_CONTROLLER ->
        -m_primaryController.getRawAxis(XboxController.Axis.kLeftX.value);
    };
    return MathUtil.applyDeadband(leftX, m_deadbandThreshold);
  }

  /** Returns the left Y axis value based on the current control scheme. */
  public Double getLeftY() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double leftY = switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        -m_primaryController.getRawAxis(1); // Mapped to W/S keys;
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        -m_primaryController.getRawAxis(LogitechConstants.Dualshock.LeftYAxis);
      case GAMESIR_CONTROLLER ->
        -m_primaryController.getRawAxis(
            frc.robot.constants.GameSirConstants.Axes.LEFT_Y);
      case XBOX_CONTROLLER ->
        -m_primaryController.getRawAxis(XboxController.Axis.kLeftY.value);
    };
    return MathUtil.applyDeadband(leftY, m_deadbandThreshold);
  }

  /** Returns the right X axis value based on the current control scheme. */
  public double getRightX() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double rightX = switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        -m_secondaryController.getRawAxis(0); // Mapped to J/L keys
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        -m_primaryController.getRawAxis(LogitechConstants.Dualshock.RightXAxis);
      case GAMESIR_CONTROLLER ->
        -m_primaryController.getRawAxis(
            frc.robot.constants.GameSirConstants.Axes.RIGHT_X);
      case XBOX_CONTROLLER ->
        -m_primaryController.getRawAxis(XboxController.Axis.kRightX.value);
    };
    return MathUtil.applyDeadband(rightX, m_deadbandThreshold);
  }

  /** Returns the right Y axis value based on the current control scheme. */
  public double getRightY() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return 0.0;
    }

    final double rightY = switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        -m_secondaryController.getRawAxis(1); // Mapped to I/K keys
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        -m_primaryController.getRawAxis(LogitechConstants.Dualshock.RightYAxis);
      case GAMESIR_CONTROLLER ->
        -m_primaryController.getRawAxis(
            frc.robot.constants.GameSirConstants.Axes.RIGHT_Y);
      case XBOX_CONTROLLER ->
        -m_primaryController.getRawAxis(XboxController.Axis.kRightY.value);
    };
    return MathUtil.applyDeadband(rightY, m_deadbandThreshold);
  }

  public boolean isXButtonPressed() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return false;
    }

    return switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 -> {
        boolean val = m_primaryController.getRawButton(2); // Mapped to X key by default
        yield val;
      }
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        m_primaryController.getRawButton(LogitechConstants.Dualshock.XButton);
      case GAMESIR_CONTROLLER ->
        m_primaryController.getRawButton(frc.robot.constants.GameSirConstants.Buttons.X);
      case XBOX_CONTROLLER ->
        m_primaryController.getRawButton(XboxController.Button.kX.value);
    };
  }

  public boolean isYButtonPressed() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return false;
    }

    return switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 -> {
        boolean val = m_primaryController.getRawButton(1); // Mapped to Z key by default
        yield val;
      }
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        m_primaryController.getRawButton(LogitechConstants.Dualshock.YButton);
      case GAMESIR_CONTROLLER ->
        m_primaryController.getRawButton(frc.robot.constants.GameSirConstants.Buttons.Y);
      case XBOX_CONTROLLER ->
        m_primaryController.getRawButton(XboxController.Button.kY.value);
    };
  }

  public boolean isAButtonPressed() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return false;
    }

    return switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        m_primaryController.getRawButton(3); // Mapped to C key by default
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        m_primaryController.getRawButton(LogitechConstants.Dualshock.AButton);
      case GAMESIR_CONTROLLER ->
        m_primaryController.getRawButton(frc.robot.constants.GameSirConstants.Buttons.A);
      case XBOX_CONTROLLER ->
        m_primaryController.getRawButton(XboxController.Button.kA.value);
    };
  }

  public boolean isBButtonPressed() {
    if (DISABLED_IN_AUTONOMOUS && DriverStation.isAutonomous()) {
      return false;
    }

    return switch (currentControlScheme) {
      case KEYBOARD1, ALT_KEYBOARD1 ->
        m_primaryController.getRawButton(4); // Mapped to V key by default
      case LOGITECH_DUALSHOCK_CONTROLLER ->
        m_primaryController.getRawButton(LogitechConstants.Dualshock.BButton);
      case GAMESIR_CONTROLLER ->
        m_primaryController.getRawButton(frc.robot.constants.GameSirConstants.Buttons.B);
      case XBOX_CONTROLLER ->
        m_primaryController.getRawButton(XboxController.Button.kB.value);
    };
  }
}