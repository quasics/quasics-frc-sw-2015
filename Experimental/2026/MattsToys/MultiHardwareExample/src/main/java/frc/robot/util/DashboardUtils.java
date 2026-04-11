package frc.robot.util;

import java.util.function.Consumer;

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

}
