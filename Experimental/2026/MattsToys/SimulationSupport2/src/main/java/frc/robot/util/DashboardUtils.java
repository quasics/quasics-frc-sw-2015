package frc.robot.util;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Some utility functions for working with the SmartDashboard/Shuffleboard,
 * including tab management.
 */
public class DashboardUtils {
  ////////////////////////////////////////////////////////////////////////////////
  //
  // Methods for adding configuration selectors and warning indicators to the
  // dashboard, which are common patterns.
  //
  // They are primarily intended to be used for development and testing purposes,
  // to allow us to easily add configuration options and warning indicators to the
  // dashboard without having to write a lot of boilerplate code each time. They
  // are also intended to be used for debugging and troubleshooting purposes, to
  // allow us to easily add indicators to the dashboard that can help us identify
  // issues and understand what is happening in the code during development and
  // testing.
  //
  // They are *not* intended to be used for telemetry that we want to display
  // during a match, since they are not optimized for performance and may have
  // some overhead associated with them.
  //

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

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Methods for managing the dashboard tabs and adding data to the dashboard,
  // with some filtering to avoid adding the same data multiple times (which
  // causes exceptions).
  //

  /**
   * Controls if sub-tabs will be allowed for the controls, or if they should all
   * go on the default "SmartDashboard" tab.
   */
  private static final boolean USE_SUB_TABS = true;

  /**
   * Controls if a special (dedicated) sub-tab will be used for the controls, or
   * if they should all go on the default "SmartDashboard" tab.
   */
  private static final boolean USE_SUB_TAB_FOR_DRIVE_TEAM = false;

  /**
   * Keeps track of the entities that have been added to the dashboard, to avoid
   * adding the same entity multiple times (which causes exceptions). The key is a
   * combination of the tab name and the label, since the same label can be used
   * on different tabs without conflict.
   */
  private static final Set<String> m_addedEntities = new HashSet<String>();

  /**
   * Helper function to generate the lookup key for the m_addedEntities set, based
   * on the tab name and the label.
   * 
   * @param tabName tab name
   * @param label   label for a control or indicator
   * @return
   */
  private static String getLookupLabel(String tabName, String label) {
    return tabName + "____" + label;
  }

  /**
   * Helper function to get the NetworkTable associated with a given tab and
   * label, if it has been added to the dashboard. This can be used to update the
   * value of a control or indicator that has already been added to the dashboard,
   * without having to keep a reference to the Sendable object that was originally
   * added.
   * 
   * If the specified tab and label combination has not been added to the
   * dashboard, this method returns null.
   * 
   * @param tabName tab name
   * @param label   label for a control or indicator
   * @return
   */
  public static NetworkTable getNetworkTable(String tabName, String label) {
    final String lookup = getLookupLabel(tabName, label);
    if (!m_addedEntities.contains(lookup)) {
      return null;
    }

    if (USE_SUB_TABS) {
      return NetworkTableInstance.getDefault()
          .getTable("Shuffleboard")
          .getSubTable(tabName)
          .getSubTable(label);
    } else {
      return NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getSubTable(label);
    }
  }

  /**
   * Helper function, providing some filtering.
   * 
   * Unlike with the SmartDashboard.put() call, you can only add an object with a
   * given name to a tab *once*; trying to do so again (even with the same object)
   * will cause an exception to be thrown. We could catch this, but that's
   * potentially expensive, so we'll instead just maintain a list that we can
   * cross-check.
   * 
   * @param tabName tab name
   * @param label   label for a control or indicator
   * @param object  the Sendable object to be added to the dashboard
   */
  private static void addToTab(String tabName, String label, Sendable object) {
    final String lookup = getLookupLabel(tabName, label);
    if (m_addedEntities.contains(lookup)) {
      // Tab already contains something with this name: not gonna add it again.
      return;
    }

    var tab = Shuffleboard.getTab(tabName);
    tab.add(label, object);
    m_addedEntities.add(lookup);
  }

  /**
   * Helper function to check if a given tab and label combination has already
   * been added to the dashboard. This can be used to avoid trying to add the same
   * control or indicator multiple times, which would cause exceptions.
   * 
   * @param tabName tab name
   * @param label   label for a control or indicator
   * @return true if the specified tab and label combination has already been
   *         added to the dashboard, false otherwise
   */
  public static boolean isPublished(String tabName, String label) {
    final String lookup = getLookupLabel(tabName, label);
    return m_addedEntities.contains(lookup);
  }

  /**
   * Helper function to publish a Sendable object to the dashboard, potentially on
   * a specific tab for drive team controls.
   * 
   * @param label  label for a control or indicator
   * @param object the Sendable object to be added to the dashboard
   */
  public static void publishForDriveTeam(String label, Sendable object) {
    if (USE_SUB_TAB_FOR_DRIVE_TEAM) {
      addToTab("Drive Team", label, object);
    } else {
      SmartDashboard.putData(label, object);
    }
  }

  /**
   * A helper function to publish a Sendable object to the dashboard, potentially
   * on a specific tab.
   * 
   * @param group  group name (tab name if using sub-tabs, or just a grouping
   *               label if not)
   * @param label  label for a control or indicator
   * @param object the Sendable object to be added to the dashboard
   */
  public static void publish(String group, String label, Sendable object) {
    if (USE_SUB_TABS) {
      addToTab(group, label, object);
    } else {
      SmartDashboard.putData(label, object);
    }
  }

  /**
   * A helper function to publish a Sendable object to the dashboard, potentially
   * on a specific tab.
   * 
   * @param group  group name (tab name if using sub-tabs, or just a grouping
   *               label if not)
   * @param object the Sendable object to be added to the dashboard
   */
  public static void publish(String group, Sendable object) {
    if (USE_SUB_TABS) {
      var tab = Shuffleboard.getTab(group);
      tab.add(object);
    } else {
      SmartDashboard.putData(object);
    }
  }
}
