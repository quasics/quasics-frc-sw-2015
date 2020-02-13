/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CommandPanel.h"

#include "Constants.h"

CommandPanel::CommandPanel() : motor(CANBusIds::VictorSpx::SpinMotor) {
  SetSubsystem("CommandPAnel");
}

// TODO(RJ): Turn these into members of the CommandPanel class, rather than
// being global variables.
rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
rev::ColorMatch m_colorMatcher;

// This method will be called once per scheduler run
void CommandPanel::Periodic() {
}

void CommandPanel::TurnWheelMotorOn() {
  bool f = true;
  if (f) {
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

  m_colorMatcher.AddColorMatch(CommandPanelConstants::kBlueTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kGreenTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kRedTarget);
  m_colorMatcher.AddColorMatch(CommandPanelConstants::kYellowTarget);

  frc::Color detectedColor = m_colorSensor.GetColor();

  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor =
      m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  // TODO(RJ): Turn the constant value for the confidence into a named constant.

  if (matchedColor == CommandPanelConstants::kBlueTarget && confidence > 90) {
    colorID = BLUE;
  } else if (matchedColor == CommandPanelConstants::kRedTarget &&
             confidence > 90) {
    colorID = RED;
  } else if (matchedColor == CommandPanelConstants::kGreenTarget &&
             confidence > 90) {
    colorID = GREEN;
  } else if (matchedColor == CommandPanelConstants::kYellowTarget &&
             confidence > 90) {
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
