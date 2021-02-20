#pragma once

#include <fstream>
#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>

#include "Constants.h"

/**
 * Wraps the handling of the configuration settings used when processing
 * the video stream on the Raspberry Pi.
 */
class VisionSettingsHelper
{
public:
  /**
   * Constructor.
   * 
   * @path filename  the path to the file used to store configuration data.
   *                 By default, this will be stored in a location relative
   *                 to the "current working directory" for the robot code
   *                 when running: for a Romi, this is the folder containing
   *                 the VS Code project; for a "big bot", this is "/" (which
   *                 is not generally a good place to store files).
   */
  VisionSettingsHelper(const std::string &filename)
      : m_filename(filename) {}

  /**
   * Returns the path to the home directory for the account that is used to
   * run code on the RoboRio.
   */
  static std::string GetSuggestedRoboRioDirectory() {
    return "/home/lvuser/";
  }

  /**
   * Returns a path to the current directory when running code for the Romi,
   * which should be the VS Code project folder.
   */
  static std::string GetSuggestedRomiDirectory() {
    return "./";
  }

  /**
   * Adds sliders for the "hue", "saturation", and "value" settings used
   * to control the color masking when processing video on the RasPi.
   * (They will be initialized to any saved values from previous runs.)
   */
  void InstallSliders()
  {
    int low_h(0), low_s(0), low_v(0),
        high_h(179), high_s(255), high_v(255);
    LoadFromFile(low_h, low_s, low_v, high_h, high_s, high_v);
    ConfigureShuffleboard(low_h, low_s, low_v, high_h, high_s, high_v);
  }

  /**
   * Writes the current vision configuration out to the file we
   * originally loaded it from (or tried to).
   * 
   * @return true on success; false on any failures
   */
  bool SaveToFile()
  {
    std::ofstream fout(m_filename);
    fout << "low_h " << int(m_lowH.GetDouble(0)) << '\n'
         << "low_s " << int(m_lowS.GetDouble(0)) << '\n'
         << "low_v " << int(m_lowV.GetDouble(0)) << '\n'
         << "high_h " << int(m_highH.GetDouble(179)) << '\n'
         << "high_s " << int(m_highS.GetDouble(255)) << '\n'
         << "high_v " << int(m_highV.GetDouble(255)) << '\n';
    fout.flush();
    return fout.good();
  }

private:
  void LoadFromFile(
      int &low_h, int &low_s, int &low_v,
      int &high_h, int &high_s, int &high_v)
  {
    std::ifstream fin(m_filename);
    std::cerr << "Loading vision settings from " << m_filename << std::endl;
    std::string key;
    int value;
    while (fin)
    {
      fin >> key >> value;
      if (fin)
      {
        if (key == "low_h")
        {
          low_h = value;
        }
        if (key == "low_s")
        {
          low_s = value;
        }
        if (key == "low_v")
        {
          low_v = value;
        }
        if (key == "high_h")
        {
          high_h = value;
        }
        if (key == "high_s")
        {
          high_s = value;
        }
        if (key == "high_v")
        {
          high_v = value;
        }
      }
    }
  }

  void ConfigureShuffleboard(
      int low_h, int low_s, int low_v,
      int high_h, int high_s, int high_v)
  {
    // Legal values for "hue" in OpenCV image processing run from 0-179.
    // On the other hand, "saturation" and "value" run from 0-255.
    wpi::StringMap<std::shared_ptr<nt::Value>> hueSliderProperties{
        {"min", nt::Value::MakeDouble(0)},
        {"max", nt::Value::MakeDouble(179)},
        {"Block increment", nt::Value::MakeDouble(1)}};
    wpi::StringMap<std::shared_ptr<nt::Value>> saturationAndValueSliderProperties{
        {"min", nt::Value::MakeDouble(0)},
        {"max", nt::Value::MakeDouble(255)},
        {"Block increment", nt::Value::MakeDouble(1)}};

    // Add sliders for each of the color-related settings.
    auto &tab = frc::Shuffleboard::GetTab(NetworkTableNames::kVisionSettingsTable);
    m_lowH = tab.Add(NetworkTableNames::kLowHSetting, low_h)
                 .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                 .WithProperties(hueSliderProperties)
                 .GetEntry();
    m_highH = tab.Add(NetworkTableNames::kHighHSetting, high_h)
                  .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                  .WithProperties(hueSliderProperties)
                  .GetEntry();
    m_lowS = tab.Add(NetworkTableNames::kLowSSetting, low_s)
                 .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                 .WithProperties(saturationAndValueSliderProperties)
                 .GetEntry();
    m_highS = tab.Add(NetworkTableNames::kHighSSetting, high_s)
                  .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                  .WithProperties(saturationAndValueSliderProperties)
                  .GetEntry();
    m_lowV = tab.Add(NetworkTableNames::kLowVSetting, low_v)
                 .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                 .WithProperties(saturationAndValueSliderProperties)
                 .GetEntry();
    m_highV = tab.Add(NetworkTableNames::kHighVSetting, high_v)
                  .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                  .WithProperties(saturationAndValueSliderProperties)
                  .GetEntry();

    // Register a "listener" function, that will be called whenever
    // someone makes changes to the settings for the vision processing.
    auto inst = nt::NetworkTableInstance::GetDefault();
    const std::string kTableBaseName("Shuffleboard/");
    auto table = inst.GetTable(kTableBaseName + NetworkTableNames::kVisionSettingsTable);
    table->AddEntryListener(
        [this](nt::NetworkTable *table, wpi::StringRef name,
               nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags) {
          if (!SaveToFile())
          {
            // Note: at least when working with the Romi, the first letter
            // is missing from the "name" data, but the name returned from
            // the entry will be stuff like "Shuffleboard/VisionSettings/Hi_H".
            std::cerr << "Warning: Failed to save vision configuration!"
                      << std::endl;
          }
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
  }

private:
  const std::string m_filename;
  nt::NetworkTableEntry m_lowH;
  nt::NetworkTableEntry m_lowS;
  nt::NetworkTableEntry m_lowV;
  nt::NetworkTableEntry m_highH;
  nt::NetworkTableEntry m_highS;
  nt::NetworkTableEntry m_highV;
};
