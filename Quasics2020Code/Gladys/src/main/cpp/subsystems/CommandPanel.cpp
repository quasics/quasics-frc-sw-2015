/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CommandPanel.h"

#include "Constants.h"

CommandPanel::CommandPanel() : motor(CANBusIds::VictorSpx::SpinMotor) {
  SetSubsystem("CommandPanel");

  m_colorMatcher.AddColorMatch(CommandPanelConstants::kBlueTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kGreenTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kRedTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kYellowTarget);
}



// This method will be called once per scheduler run
void CommandPanel::Periodic() {
}

void CommandPanel::TurnWheelMotorOn(bool f) {
  
    if(f){
    motor.Set(0.5);
  } else {
    motor.Set(-0.5);
  }
}
void CommandPanel::TurnWheelMotorOff() {
  motor.Set(0);
}

CommandPanel::Color CommandPanel::getCurrentColor() {
  Color colorID = UNKNOWN;

  const frc::Color detectedColor = m_colorSensor.GetColor();

  std::string colorString;
  double confidence = 0.0;
  const frc::Color matchedColor =
      m_colorMatcher.MatchClosestColor(detectedColor, confidence);

\  constexpr double kMinimumConfidence = 0.90;

  if (matchedColor == CommandPanelConstants::kBlueTarget && confidence > kMinimumConfidence) {
    colorID = BLUE;
  } else if (matchedColor == CommandPanelConstants::kRedTarget &&
             confidence > kMinimumConfidence) {
    colorID = RED;
  } else if (matchedColor == CommandPanelConstants::kGreenTarget &&
             confidence > kMinimumConfidence) {
    colorID = GREEN;
  } else if (matchedColor == CommandPanelConstants::kYellowTarget &&
             confidence > kMinimumConfidence) {
    colorID = YELLOW;
  } else {
    colorID = UNKNOWN;
  }

  return colorID;
}

std::string CommandPanel::getColorName(Color c) {
  std::string colorName = "";
  switch (c) {
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
