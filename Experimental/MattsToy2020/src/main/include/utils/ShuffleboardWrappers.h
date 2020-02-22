#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include <iostream>
#include <sstream>
#include <string>

namespace ShuffleboardWrappers {

  class SimpleDisplay {
   public:
    explicit SimpleDisplay(const std::string& title,
                           const std::string& tabName = std::string())
        : SimpleDisplay("", title, tabName) {
    }
    SimpleDisplay(const std::string defaultValue, const std::string& title,
                  const std::string& tabName) {
      frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(tabName);
      entry = tab.Add(title, "")
                  .WithWidget(frc::BuiltInWidgets::kTextView)
                  .GetEntry();
    }

    /**
     * Sets a new value to be displayed.
     *
     * @param value  the new value to be displayed
     */
    template <typename T>
    void SetValue(T value) {
      std::ostringstream sout;
      sout << value;
      std::string text = sout.str();
      entry.SetString(sout.str());
    }

   private:
    /// NetworkTable entry associated with the slider.
    nt::NetworkTableEntry entry;
  };

  // class NumberBar {
  //  public:
  //   NumberBar(double min, double max, const std::string& title,
  //                const std::string& tabName = std::string()) {
  //     frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(tabName);
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
                           const std::string& tabName = std::string())
        : BooleanToggle(false, title, tabName) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the default/initial value for the control
     * @param title   title shown for the toggle control on the dashboard
     * @param tabName name of the tab on which the toggle should be placed; if
     *                unspecified, it will be put on the default
     *                "SmartDashboard" tab
     */
    BooleanToggle(bool defaultValue, const std::string& title,
                  const std::string& tabName) {
      frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(tabName);
      entry = tab.Add(title, defaultValue)
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