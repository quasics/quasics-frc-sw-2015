// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for common dashboard-related tasks, such as adding selectors
 * and warning indicators.
 */
public class DashboardUtils {
  /**
   * Utility method for adding simple value selectors (with a callback) to the
   * dashboard.
   * 
   * @param <T>          underlying data type for selections
   * @param tabName      name of the tab on the dashboard to which the selector
   *                     will be added
   * @param array        array of values to show to the user
   * @param defaultValue default value (to be initially selected)
   * @param label        label associated with the selector
   * @param onChange     callback to be invoked when the selection is changed
   */
  public static <T> void addConfigSelector(String tabName, T[] array, T defaultValue, String label,
      Consumer<T> onChange) {
    final SendableChooser<T> chooser = new SendableChooser<>();
    for (var element : array) {
      if (defaultValue.equals(element)) {
        chooser.setDefaultOption(String.valueOf(element), element);
      } else {
        chooser.addOption(String.valueOf(element), element);
      }
    }

    if (tabName != null && tabName.trim().length() > 0) {
      Shuffleboard.getTab(tabName).add(label, chooser);
    } else {
      SmartDashboard.putData(label, chooser);
    }
    chooser.onChange(onChange);
  }

  /**
   * A utility method for adding a warning indicator to the dashboard.
   * 
   * The indicator consists of a boolean box (which changes color based on the
   * boolean value) and a text view (which displays a warning message when the
   * boolean value is false).
   * 
   * The method returns a Consumer<Boolean> that can be used to update the status
   * of the warning indicator. When the consumer is called with a boolean value,
   * it updates the boolean box and the text view accordingly.
   * 
   * The text for the warning message is supplied by the textSupplier, which is
   * called each time the consumer is invoked to get the latest warning message.
   * 
   * @param tabName      name of the tab on the dashboard to which the warning
   *                     indicator will be added
   * @param lightTitle   name associated with the boolean box (status light)
   * @param textTitle    name associated with the text view (warning message)
   * @param textSupplier supplies the text to be displayed in the warning message;
   *                     this is called each time the consumer is invoked to get
   *                     the latest warning message
   * @return a Consumer<Boolean> that can be used to update the status of the
   *         warning indicator
   */
  static public Consumer<Boolean> addWarningIndicator(String tabName, String lightTitle, String textTitle,
      Supplier<String> textSupplier) {
    GenericEntry statusLightEntry = Shuffleboard.getTab(tabName)
        .add(lightTitle, true)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0)
        .withSize(4, 1)
        .withProperties(Map.of("Color when true", "Green", "Color when false", "Red"))
        .getEntry();
    GenericEntry hiddenWarningEntry = Shuffleboard.getTab(tabName)
        .add(textTitle, "")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 0)
        .withSize(4, 1)
        // This is the "highlight" — set the background to a specific color
        .withProperties(Map.of("Background Color", "Orange"))
        .getEntry();
    hiddenWarningEntry.setString(textSupplier.get());

    return (Boolean b) -> {
      statusLightEntry.setBoolean(b);
      hiddenWarningEntry.setString(textSupplier.get());
    };
  }
}
