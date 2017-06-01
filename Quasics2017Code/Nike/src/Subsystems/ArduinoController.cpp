#include "ArduinoController.h"
#include "../RobotMap.h"

std::shared_ptr<SerialPort> serialPort;
ArduinoController::ArduinoController() :
		Subsystem("ExampleSubsystem") {
	serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
	currentColor = kGreen;
	currentBrightness = kOff;
}

void ArduinoController::SetLightColor(ColorMode colorMode) {
	std::string output = "";
	currentColor = colorMode;

	switch (colorMode) {
	case kRed:
		output = "red;";
		break;
	case kBlue:
		output = "blue;";
		break;
	case kWhite:
		output = "white;";
		break;
	case kRainbow:
		output = "rainbow;";
		break;
	case kRainbowReverse:
		output = "rainbowreverse;";
		break;
	case kOldCycle:
		output = "oldcycle;";
		break;
	case kYellow:
		output = "yellow;";
		break;
	case kQuasics:
		output = "quasics;";
		break;
	case kBrown:
		output = "brown;";
		break;
	case kPurple:
		output = "purple;";
		break;
	case kGreen:
	default:
		output = "green;";
		break;
	}
	serialPort->Write(output);
}
void ArduinoController::SetLightDynamic(BrightnessMode brightnessMode) {
	std::string output = "";
	currentBrightness = brightnessMode;

	switch (brightnessMode){
	case kOn:
		output = "on;";
		break;
	case kBreathing:
		output = "Breathing;";
		break;
	case kBlinking:
		output = "Blinking;";
		break;
	case kDashed:
		output = "Dashed;";
		break;
	case kRollIn:
		output = "RollIn;";
		break;
	case kRollOut:
		output = "RollOut;";
		break;
	case kRolling:
		output = "Rolling;";
		break;
	case kSnakeIn:
		output = "SnakeIn;";
		break;
	case kSnakeOut:
		output = "SnakeOut;";
		break;
	case kSnake:
		output = "Snake;";
		break;
	default:
		output = "off;";
	}

	serialPort->Write(output);

}

ArduinoController::ColorMode ArduinoController::GetColorMode() {
	return currentColor;
}

ArduinoController::BrightnessMode ArduinoController::GetBrightnessMode() {
	return currentBrightness;
}

void ArduinoController::InitDefaultCommand() {
// Set the default command for a subsystem here.
// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
