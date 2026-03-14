package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Some utility functions for working with the SmartDashboard/Shuffleboard,
 * including tab management.
 */
public class DashboardUtils {
  public static boolean USE_SUB_TABS = true;
  public static boolean USE_SUB_TAB_FOR_DRIVE_TEAM = false;

  private static Set<String> m_addedEntities = new HashSet<String>();

  private static String getLookupLabel(String tabName, String label) {
    return tabName + "____" + label;
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
   * @param tabName
   * @param label
   * @param object
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

  public static boolean isPublished(String tabName, String label) {
    final String lookup = getLookupLabel(tabName, label);
    return m_addedEntities.contains(lookup);
  }

  public static void publishForDriveTeam(String label, Sendable object) {
    if (USE_SUB_TAB_FOR_DRIVE_TEAM) {
      addToTab("Drive Team", label, object);
    } else {
      SmartDashboard.putData(label, object);
    }
  }

  public static void publish(String group, String label, Sendable object) {
    if (USE_SUB_TABS) {
      addToTab(group, label, object);
    } else {
      SmartDashboard.putData(label, object);
    }
  }

  public static void publish(String group, Sendable object) {
    if (USE_SUB_TABS) {
      var tab = Shuffleboard.getTab(group);
      tab.add(object);
    } else {
      SmartDashboard.putData(object);
    }
  }
}
