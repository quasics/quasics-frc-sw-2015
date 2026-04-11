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
