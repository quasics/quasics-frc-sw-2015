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
  enum MotorType {
    SparkPwm,
    SparkMax,
    TalonFX,
    ThriftyNova,
  }

  private static final String MOTOR_TYPE_PREF_KEY = "MotorType";
  private static final String MOTOR_ID_PREF_KEY = "MotorId";
  private static final String MOTOR_INVERTED_PREF_KEY = "MotorInverted";

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

  private static int getMotorIdFromPreferences(int defaultValue) {
    return Preferences.getInt(MOTOR_ID_PREF_KEY, defaultValue);
  }

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

  private static final int DEFAULT_MOTOR_ID = 1;
  private static final MotorType DEFAULT_MOTOR_TYPE = MotorType.SparkMax;
  private static final boolean DEFAULT_MOTOR_INVERTED = false;

  private static final MotorType SELECTED_MOTOR_TYPE = getMotorTypeFromPreferences(DEFAULT_MOTOR_TYPE);
  private static final int SELECTED_MOTOR_ID = getMotorIdFromPreferences(DEFAULT_MOTOR_ID);
  private static final boolean SELECTED_MOTOR_INVERTED = getInvertedFromPreferences(DEFAULT_MOTOR_INVERTED);

  // Sets up a "single motor thing", based on the selected hardware
  // configuration.
  final ISingleMotorThing m_singleMotorThing = allocateSingleMotorThing(
      SELECTED_MOTOR_TYPE,
      SELECTED_MOTOR_ID,
      SELECTED_MOTOR_INVERTED);

  /** Constructor. */
  public RobotContainer() {
    Shuffleboard.getTab(VariableSpeedCommand.TAB_NAME)
        .add("Variable Speed", new VariableSpeedCommand(m_singleMotorThing))
        .withWidget(BuiltInWidgets.kCommand);

    addPowerButton("Stop!", 0);
    addPowerButton("-10% power", -.10);
    addPowerButton("-25% power", -.25);
    addPowerButton("-50% power", -.5);
    addPowerButton("-100% power", -1.0);
    addPowerButton("+10% power", +.10);
    addPowerButton("+25% power", +.25);
    addPowerButton("+50% power", +.5);
    addPowerButton("+100% power", +1.0);

    SmartDashboard.putData(m_singleMotorThing.asSendable());
    setupSubsystemConfigSelectors();
  }

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

  private GenericEntry hiddenWarningEntry = null;
  private GenericEntry statusLightEntry;
  static final String CONFIG_TAB_NAME = "Config";

  <T> void addConfigSelector(T[] array, T defaultValue, String label, Consumer<T> onChange) {
    if (hiddenWarningEntry == null) {
      statusLightEntry = Shuffleboard.getTab(CONFIG_TAB_NAME)
          .add("Reset needed", true)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(0, 0)
          .withSize(4, 1)
          .withProperties(Map.of("Color when true", "Green", "Color when false", "Red"))
          .getEntry();
      hiddenWarningEntry = Shuffleboard.getTab(CONFIG_TAB_NAME)
          .add("Restart Alert", "")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(1, 0)
          .withSize(4, 1)
          // This is the "highlight" — set the background to a specific color
          .withProperties(Map.of("Background Color", "Orange"))
          .getEntry();
      hiddenWarningEntry.setString("");
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
          System.out.println("Signal the user to restart");
          hiddenWarningEntry.setString("Restart the robot for changes to take effect.");
          statusLightEntry.setBoolean(false);
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
