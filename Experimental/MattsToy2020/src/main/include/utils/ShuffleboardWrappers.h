#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include <string>

namespace ShuffleboardWrappers {

  inline frc::ShuffleboardTab& getTab(const std::string& tabName) {
    return frc::Shuffleboard::GetTab(tabName.empty() ? "SmartDashboard"
                                                     : tabName);
  }

  // class NumberBar {
  //  public:
  //   NumberBar(double min, double max, const std::string& title,
  //                const std::string& tabName = std::string()) {
  //     frc::ShuffleboardTab& tab = getTab(tabName);
  //     entry = tab.Add(title, false)
  //                 .WithWidget(frc::BuiltInWidgets::kNumberBar)
  //                 .GetEntry();
  //   }
  //
  //  private:
  //   /// NetworkTable entry associated with the slider.
  //   nt::NetworkTableEntry entry;
  // };

  /**
   * Utility class, simplifying the task of putting an "on/off" toggle switch on
   * the Smart Dashboard/Shuffleboard, which can be read by the robot code.
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
    explicit BooleanToggle(const std::string& title,
                           const std::string& tabName = std::string()) {
      frc::ShuffleboardTab& tab = getTab(tabName);
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

    /**
     * Sets a new value for the toggle (e.g., if the robot has decided it can't
     * comply with the user's request).
     *
     * @param value  the new value for the toggle
     */
    void SetValue(bool value) {
      entry.SetBoolean(value);
    }

   private:
    /// NetworkTable entry associated with the toggle.
    nt::NetworkTableEntry entry;
  };

}  // namespace ShuffleboardWrappers