// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.IntStream;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.VariableSpeedCommand;
import frc.robot.subsystems.interfaces.ISingleMotorThing;
import frc.robot.subsystems.real.SingleMotorThingNova;
import frc.robot.subsystems.real.SingleMotorThingSpark;
import frc.robot.subsystems.real.SingleMotorThingTalonPwm;
import frc.robot.util.DashboardUtils;

/**
 * The container for the robot. Contains/configures subsystems, OI devices, and
 * commands.
 */
public class RobotContainer {
  /**
   * Enum representing the different types of motors/configurations that can be
   * used.
   */
  enum MotorType {
    /** PWM-controlled SparkMax. */
    SparkPwm,
    /** CAN-controlled SparkMax. */
    SparkMax,
    /** CAN-controlled TalonFX. */
    TalonFX,
    /** CAN-controlled ThriftyNova. */
    ThriftyNova,
  }

  /** Preference key for motor type configuration setting. */
  private static final String MOTOR_TYPE_PREF_KEY = "MotorType";
  /** Preference key for motor ID configuration setting. */
  private static final String MOTOR_ID_PREF_KEY = "MotorId";
  /** Preference key for motor inversion configuration setting. */
  private static final String MOTOR_INVERTED_PREF_KEY = "MotorInverted";

  /**
   * Utility method for reading the motor type from preferences, with error
   * handling to fall back to a default value if the preference is not set or is
   * invalid.
   * 
   * @param defaultValue default motor type to use if the preference is not set or
   *                     is invalid
   * @return the motor type read from preferences, or the default value if invalid
   */
  private static MotorType getMotorTypeFromPreferences(MotorType defaultValue) {
    String motorTypeString = Preferences.getString(MOTOR_TYPE_PREF_KEY, defaultValue.name());
    try {
      return MotorType.valueOf(motorTypeString);
    } catch (IllegalArgumentException e) {
      System.err.println("Invalid MotorType in preferences: " + motorTypeString
          + ". Defaulting to " + defaultValue.name() + ".");
      return defaultValue;
    }
  }

  /**
   * Utility method for reading the motor ID from preferences, with error
   * handling to fall back to a default value if the preference is not set or is
   * invalid.
   * 
   * @param defaultValue default motor ID to use if the preference is not set or
   *                     is invalid
   * @return the motor ID read from preferences, or the default value if invalid
   */
  private static int getMotorIdFromPreferences(int defaultValue) {
    return Preferences.getInt(MOTOR_ID_PREF_KEY, defaultValue);
  }

  /**
   * Utility method for reading the motor inversion setting from preferences, with
   * error handling to fall back to a default value if the preference is not set
   * or is invalid.
   * 
   * @param defaultValue default motor inversion setting to use if the preference
   *                     is not set or is invalid
   * @return the motor inversion setting read from preferences, or the default
   *         value if invalid
   */
  private static boolean getInvertedFromPreferences(boolean defaultValue) {
    return Preferences.getBoolean(MOTOR_INVERTED_PREF_KEY, defaultValue);
  }

  /**
   * Allocates a "single motor thing" based on the specified motor type, ID, and
   * inversion setting.
   * 
   * This method can be used to create "single motor things" with different
   * hardware configurations (e.g., based on preferences or other runtime
   * conditions) without needing to modify the main RobotContainer constructor or
   * create multiple fixed configurations in the HardwareConfig enum.
   * 
   * @param motorType target motor type
   * @param motorId   ID for the motor (e.g., PWM channel, CAN ID, etc., depending
   *                  on the motor type)
   * @param inverted  whether the motor should be inverted
   * @return an ISingleMotorThing instance configured according to the specified
   *         parameters
   */
  static ISingleMotorThing allocateSingleMotorThing(MotorType motorType, int motorId, boolean inverted) {
    System.out.println("Allocating single motor thing: " + motorType + ", ID: " + motorId + ", Inverted: " + inverted);
    return switch (motorType) {
      case SparkPwm -> new SingleMotorThingSpark(motorId, inverted);
      case SparkMax -> new SingleMotorThingSpark(motorId, inverted);
      case TalonFX -> new SingleMotorThingTalonPwm(motorId, inverted);
      case ThriftyNova -> new SingleMotorThingNova(motorId, inverted);
    };
  }

  /**
   * Default motor type to use if the preference is not set or is invalid.
   */
  private static final MotorType DEFAULT_MOTOR_TYPE = MotorType.SparkMax;

  /**
   * Default motor ID (CAN, PWM, etc.) to use if the preference is not set or is
   * invalid.
   */
  private static final int DEFAULT_MOTOR_ID = 1;

  /**
   * Default motor inversion setting to use if the preference is not set or is
   * invalid.
   */
  private static final boolean DEFAULT_MOTOR_INVERTED = false;

  /** Selected motor type read from preferences. */
  private static final MotorType SELECTED_MOTOR_TYPE = getMotorTypeFromPreferences(DEFAULT_MOTOR_TYPE);

  /** Selected motor ID read from preferences. */
  private static final int SELECTED_MOTOR_ID = getMotorIdFromPreferences(DEFAULT_MOTOR_ID);

  /** Selected motor inversion setting read from preferences. */
  private static final boolean SELECTED_MOTOR_INVERTED = getInvertedFromPreferences(DEFAULT_MOTOR_INVERTED);

  /**
   * The "single motor thing" subsystem, allocated based on the selected motor
   * type, ID, and inversion setting from preferences.
   */
  final ISingleMotorThing m_singleMotorThing = allocateSingleMotorThing(
      SELECTED_MOTOR_TYPE,
      SELECTED_MOTOR_ID,
      SELECTED_MOTOR_INVERTED);

  /** Constructor. */
  public RobotContainer() {
    // Simple power control buttons for testing the motor.
    addPowerButton("Stop!", 0);
    addPowerButton("-10% power", -.10);
    addPowerButton("-25% power", -.25);
    addPowerButton("-50% power", -.5);
    addPowerButton("-100% power", -1.0);
    addPowerButton("+10% power", +.10);
    addPowerButton("+25% power", +.25);
    addPowerButton("+50% power", +.5);
    addPowerButton("+100% power", +1.0);

    // Add the single motor thing to the dashboard so we can see its status and
    // control it with any built-in controls we set up in the subsystem.
    SmartDashboard.putData(m_singleMotorThing.asSendable());

    // Variable-speed control for testing the motor, with a slider to specify the
    // speed.
    Shuffleboard.getTab(VariableSpeedCommand.TAB_NAME)
        .add("Variable Speed", new VariableSpeedCommand(m_singleMotorThing))
        .withWidget(BuiltInWidgets.kCommand);

    // "Subsystem configuration" tab setup.
    setupSubsystemConfigSelectors();
  }

  /**
   * Set up configuration selectors for motor type, ID, and inversion, with
   * callbacks that update the preferences and signal the user to restart the
   * robot for changes to take effect.
   */
  private void setupSubsystemConfigSelectors() {
    addConfigSelector(
        new Boolean[] { true, false },
        SELECTED_MOTOR_INVERTED,
        "Motor Inverted",
        newValue -> {
          System.out.println("Motor Inverted changed to: " + newValue);
          Preferences.setBoolean(MOTOR_INVERTED_PREF_KEY, newValue);
        });

    addConfigSelector(
        MotorType.values(),
        SELECTED_MOTOR_TYPE,
        "Motor Type",
        newValue -> {
          System.out.println("Motor Type changed to: " + newValue);
          Preferences.setString(MOTOR_TYPE_PREF_KEY, newValue.name());
        });

    addConfigSelector(
        IntStream.rangeClosed(1, 15)
            .boxed()
            .toArray(Integer[]::new),
        SELECTED_MOTOR_ID,
        "Motor Id",
        newValue -> {
          System.out.println("Motor Id changed to: " + newValue);
          Preferences.setInt(MOTOR_ID_PREF_KEY, newValue);
        });
  }

  /** Name for the configuration tab. */
  static final String CONFIG_TAB_NAME = "Config";

  /**
   * Warning trigger for configuration changes; will update the configuration tab
   * to indicate that a restart is needed.
   */
  private Consumer<Boolean> warningTrigger;

  /**
   * Whether a restart is needed for configuration changes to take effect.
   */
  private boolean m_restartNeeded = false;

  /**
   * Utility method for adding a configuration selector to the dashboard, with a
   * callback that updates the configuration and signals the user to restart if
   * necessary.
   * 
   * Note that this method is specific to this example, since it assumes that any
   * configuration change will require a restart, and it also assumes that the
   * warning indicator is set up in a specific way. However, it demonstrates how
   * to create a reusable utility method that encapsulates common patterns for
   * configuration selectors and user feedback.
   * 
   * In a more complex robot codebase, you might want to create a more flexible
   * and robust configuration management system, but this serves as a simple
   * example of how to handle configuration changes and user feedback in a
   * consistent way.
   * 
   * This method also "lazily initializes" the warning trigger the first time it's
   * needed, which allows us to avoid setting up the warning indicator if no
   * configuration selectors are added.
   * 
   * @param <T>          underlying data type for selections
   * @param array        array of values to show to the user
   * @param defaultValue default value (to be initially selected)
   * @param label        label associated with the selector
   * @param onChange     callback to be invoked when the selection is changed;
   *                     should update the relevant configuration based on the new
   *                     value
   */
  <T> void addConfigSelector(T[] array, T defaultValue, String label, Consumer<T> onChange) {
    if (warningTrigger == null) {
      warningTrigger = DashboardUtils.addWarningIndicator(CONFIG_TAB_NAME, "Reset needed", "Restart alert", () -> {
        return m_restartNeeded ? "Restart the robot for changes to take effect." : "";
      });
    }
    DashboardUtils.addConfigSelector(
        CONFIG_TAB_NAME,
        array,
        defaultValue,
        label,
        value -> {
          // Post the new value back to the callback.
          onChange.accept(value);

          // Changing the configuration typically requires re-allocating hardware.
          m_restartNeeded = true;
          System.out.println("Signal the user to restart");
          warningTrigger.accept(false);
        });
  }

  private void addPowerButton(String label, double percent) {
    SmartDashboard.putData(label,
        new FunctionalCommand(
            // onInit (can't be null)
            () -> {
              m_singleMotorThing.setSpeed(percent);
            },
            // onExecute (can't be null)
            () -> {
              // No-op: speed was set in initialization
            },
            // onEnd (can't be null)
            (Boolean b) -> {
              m_singleMotorThing.stop();
            },
            // isFinished (can't be null)
            () -> false,
            // Dependency
            m_singleMotorThing.asSubsystem()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
