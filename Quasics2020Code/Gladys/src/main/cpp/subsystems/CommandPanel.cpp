/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CommandPanel.h"

  constexpr int SpinMotor = 7;
CommandPanel::CommandPanel():motor(SpinMotor){}
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
// This method will be called once per scheduler run
void CommandPanel::Periodic() {}

//if mode is 0, no turbo, else TURBO!!!

void CommandPanel::TurnWheelMotorOn() {
    motor.Set(0.5);
}
void CommandPanel::TurnWheelMotorOff() {
    motor.Set(0);
}
CommandPanel::Color CommandPanel::getCurrentColor(){
    Color colorID = UNKNOWN;
    
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);

    frc::Color detectedColor = m_colorSensor.GetColor();

     std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor =
        m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget && confidence>90) {
      colorID = BLUE;
    } else if (matchedColor == kRedTarget && confidence>90) {
      colorID = RED;
    } else if (matchedColor == kGreenTarget && confidence>90) {
      colorID = GREEN;
    } else if (matchedColor == kYellowTarget && confidence>90) {
      colorID = YELLOW;
    } else {
      colorID = UNKNOWN;
    }

    return colorID;
}

std::string CommandPanel::getColorName(Color c){
    std::string colorName="";
    switch (c)
    {
    case BLUE:
        colorName = "Blue";
        break;
    case RED:
        colorName = "Red";
        break;
    case GREEN:
        colorName = "Green";
        break;
    case YELLOW:
        colorName = "Yellow";
        break;
    case UNKNOWN:
        colorName = "Unknown";
        break;
    default:
        colorName = "ERROR! ERROR! ERROR! Does not compute.";
        break;
    }
    return colorName;
}
