#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include <string>

namespace ShuffleboardWrappers {

  /**
   * Utility class, simplifying the task of putting an "on/off" toggle on the
   * Smart Dashboard/Shuffleboard, which can be read by the robot code.
   */
  class BooleanToggle {
   public:
    /**
     * Constructor.
     *
     * @param title   title shown for the toggle control on the dashboard
     * @param tabName name of the tab on which the toggle should be placed; if
     *                unspecified, it will be put on the default
     *                "SmartDashboard" tab
     */
    explicit BooleanToggle(std::string title, std::string tabName = "") {
      frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(
          tabName.empty() ? "SmartDashboard" : tabName);
      entry = tab.Add(title, false)
                  .WithWidget(frc::BuiltInWidgets::kToggleSwitch)
                  .GetEntry();
    }

    /**
     * Returns the current state of the toggle (true == on, false == off).
     *
     * @param defaultValue  if the toggle can't be found/read, return this value
     */
    bool GetValue(bool defaultValue = false) {
      return entry.GetBoolean(defaultValue);
    }

   private:
    /// NetworkTable entry associated with the toggle.
    nt::NetworkTableEntry entry;
  };

}  // namespace ShuffleboardWrappers