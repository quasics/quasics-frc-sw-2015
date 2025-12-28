package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LogitechConstants;
import frc.robot.constants.OperatorConstants;

public final class DriverJoystickWrapper {
  private final Joystick m_driveController;
  private final boolean m_saveToPreferences;

  public DriverJoystickWrapper(int joystickId) {
    this(joystickId, true);
  }

  public DriverJoystickWrapper(int joystickId, boolean saveToPreferences) {
    m_driveController = new Joystick(OperatorConstants.DRIVER_JOYSTICK_ID);
    m_saveToPreferences = saveToPreferences;

    addDriveControlSelectionToSmartDashboard();
  }

  /** Enumeration of available drive control schemes. */
  public enum ControllerType {
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
  private ControllerType currentControlScheme = ControllerType.KEYBOARD1;

  /** Returns the currently selected control scheme. */
  public ControllerType getCurrentControlScheme() {
    return currentControlScheme;
  }

  /** Sets up the drive control selection on the SmartDashboard. */
  private void addDriveControlSelectionToSmartDashboard() {
    // Load the last-selected control scheme from preferences
    int savedControlSchemeOrdinal = m_saveToPreferences
        // Load the last-selected control scheme from preferences
        ? Preferences.getInt("DriveControlScheme", 0)
        // Default to 0 if not working with preferences
        : 0;
    if (savedControlSchemeOrdinal < 0 || savedControlSchemeOrdinal >= ControllerType.values().length) {
      savedControlSchemeOrdinal = 0;
    }
    currentControlScheme = ControllerType.values()[savedControlSchemeOrdinal];

    // Build/install the chooser, establishing the saved scheme as the default
    SendableChooser<ControllerType> driveInputChooser = new SendableChooser<ControllerType>();
    for (var option : ControllerType.values()) {
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
  private void updateControlScheme(ControllerType controlScheme) {
    currentControlScheme = controlScheme;
    System.out.println("Control scheme set to: " + controlScheme.getControlSchemeName());

    if (m_saveToPreferences) {
      Preferences.setInt("DriveControlScheme", controlScheme.ordinal());
    }
  }

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
    return MathUtil.applyDeadband(forward, OperatorConstants.DEADBAND_THRESHOLD);
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
    return MathUtil.applyDeadband(rotation, OperatorConstants.DEADBAND_THRESHOLD);
  }

}
