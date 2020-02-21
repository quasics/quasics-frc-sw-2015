#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include <string>

namespace ShuffleboardWrappers {
  class BooleanToggle {
   public:
    explicit BooleanToggle(std::string name, std::string tabName = "") {
      frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(
          tabName.empty() ? "SmartDashboard" : tabName);
      entry = tab.Add(name, false)
                  .WithWidget(frc::BuiltInWidgets::kToggleSwitch)
                  .GetEntry();
    }
    bool GetValue(bool defaultValue = false) {
      return entry.GetBoolean(defaultValue);
    }

   private:
    nt::NetworkTableEntry entry;
  };

}  // namespace ShuffleboardWrappers