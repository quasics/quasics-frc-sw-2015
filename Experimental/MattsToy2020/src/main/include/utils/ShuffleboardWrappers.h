/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Quasics Robotics and Matthew J. Healy                   */
/* All Rights Reserved.                                                       */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the BSD license file in the root directory of the   */
/* project repository.                                                        */
/*----------------------------------------------------------------------------*/

/**
 * @file
 *
 * This file provides a collection of helper classes, meant to make it easier to
 * display information/controls on the "Shuffleboard" interface for the FRC
 * Driver Station software.
 */

#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include <functional>
#include <iostream>
#include <sstream>
#include <string>

namespace ShuffleboardWrappers {

  /**
   * Defines a container (list) widget on a Shuffleboard tab, which can be used
   * to group other widgets together in a logical (and more space-efficient)
   * fashion.
   */
  class Collection {
   public:
    /**
     * Constructs a list to contain other widgets on a Shuffleboard tab.
     *
     * @param title   the title shown for the list of components (e.g., "Encoder
     *                Values")
     * @param tabName the name of the tab on which the list should be placed
     */
    Collection(const std::string& title,
               const std::string& tabName = std::string())
        : layout(frc::Shuffleboard::GetTab(tabName).GetLayout(
              title, frc::BuiltInLayouts::kList)) {
    }

    /// Returns the underlying Shuffleboard widget.
    frc::ShuffleboardLayout& Get() {
      return layout;
    }

   private:
    /// The underlying Shuffleboard widget used to manage the list.
    frc::ShuffleboardLayout& layout;
  };

  /**
   * A class to display read-only (textual/numeric) values on a Shuffleboard
   * tab.
   */
  class SimpleDisplay {
   public:
    /**
     * Constructor, initially displaying an empty string.
     *
     * @param title   the title shown for the data (e.g., "Left Encoder
     *                (inches)")
     * @param tabName the name of the tab on which the widget should be placed
     */
    explicit SimpleDisplay(const std::string& title,
                           const std::string& tabName = std::string())
        : SimpleDisplay("", title, tabName) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the initial data to be displayed
     * @param title   the title shown for the data (e.g., "Left Encoder
     *                (inches)")
     * @param tabName the name of the tab on which the widget should be placed
     */
    SimpleDisplay(const std::string& defaultValue, const std::string& title,
                  const std::string& tabName)
        : SimpleDisplay(defaultValue, title,
                        frc::Shuffleboard::GetTab(tabName)) {
    }

    /**
     * Constructor.
     *
     * @param title   the title shown for the data (e.g., "Left Encoder
     *                (inches)")
     * @param collection the collection in which the data should be placed
     */
    SimpleDisplay(const std::string& title, Collection& collection)
        : SimpleDisplay("", title, collection) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the initial data to be displayed
     * @param title   the title shown for the data (e.g., "Left Encoder
     *                (inches)")
     * @param collection the collection in which the data should be placed
     */
    SimpleDisplay(const std::string& defaultValue, const std::string& title,
                  Collection& collection)
        : SimpleDisplay(defaultValue, title, collection.Get()) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the initial data to be displayed
     * @param title   the title shown for the data (e.g., "Left Encoder
     *                (inches)")
     * @param container the Shuffleboard container in which the data should be
     *                  placed
     */
    SimpleDisplay(const std::string& defaultValue, const std::string& title,
                  frc::ShuffleboardContainer& container) {
      entry = container.Add(title, "")
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
     * @param title   title shown for the toggle control on the dashboard (e.g.,
     *                "Debug on")
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
     * Constructor, setting the innitial state of the toggle to "off".
     *
     * @param title   the title shown for the toggle (e.g., "Debug on")
     * @param collection the collection in which the data should be placed
     */
    BooleanToggle(const std::string& title, Collection& container)
        : BooleanToggle(false, title, container) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the initial state for the toggle (true == on)
     * @param title   the title shown for the toggle (e.g., "Debug on")
     * @param collection the collection in which the data should be placed
     */
    BooleanToggle(bool defaultValue, const std::string& title,
                  Collection& container)
        : BooleanToggle(defaultValue, title, container.Get()) {
    }

    /**
     * Constructor.
     *
     * @param defaultValue the initial state for the toggle (true == on)
     * @param title   the title shown for the toggle (e.g., "Debug on")
     * @param container the Shuffleboard container in which the data should be
     *                  placed
     */
    BooleanToggle(bool defaultValue, const std::string& title,
                  frc::ShuffleboardContainer& container) {
      entry = container.Add(title, defaultValue)
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
