package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardUtils {
  public static boolean USE_SUB_TABS = true;
  public static boolean USE_SUB_TAB_FOR_DRIVE_TEAM = false;

  private static Set<String> m_addedEntities = new HashSet<String>();

  private static void addToTab(String tabName, String label, Sendable object) {
    String lookup = tabName + "____" + label;
    if (m_addedEntities.contains(lookup)) {
      return;
    }

    var tab = Shuffleboard.getTab(tabName);
    tab.add(label, object);
    m_addedEntities.add(lookup);
  }

  public static boolean isPublished(String tabName, String label) {
    String lookup = tabName + "____" + label;
    return m_addedEntities.contains(label);
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
