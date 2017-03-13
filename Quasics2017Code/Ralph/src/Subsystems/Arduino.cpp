#include "Arduino.h"
#include "../RobotMap.h"
#include <string.h>

Arduino::Arduino() :
		Subsystem("ExampleSubsystem") {
	serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
}

void Arduino::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

std::string Arduino::GetCameraData() {
	char buffer[200] = "";
	std::string returnString = "";
	if (serialPort->GetBytesReceived() > 0) {
		uint32_t bytesRead = 0;
		bytesRead = serialPort->Read(buffer, sizeof(buffer) - 1);
		if (bytesRead >= 0) {
			buffer[bytesRead] = 0;
		}

		bool isTerminated = false;
		for (uint32_t i = 0; i < bytesRead || isTerminated;i++){
			returnString += buffer[i];
			if (buffer[i] == ';'){
				isTerminated = true;
			}
		}
	}

	return returnString;
}

void Arduino::SetLEDColor(ColorMode whichColor) {
	std::string PrintString = ";";
	switch (whichColor) {
	case kRed:
		PrintString = "Red";
		break;
	case kGreen:
		PrintString = "Green";
		break;
	case kBlue:
		PrintString = "Blue";
		break;
	case kWhite:
		PrintString = "White";
		break;
	case kRainbow:
		PrintString = "Rainbow";
		break;
	case kOldCycle:
		PrintString = "OldCycle";
		break;
	case kYellow:
		PrintString = "Yellow";
		break;
	case kQuasics:
		PrintString = "Quasics";
		break;
	case kBrown:
		PrintString = "Brown";
		break;
	default:
		PrintString = "NoColor";
		break;
	}
	serialPort->Write(PrintString);
}

void Arduino::SetBrightnessMode(BrightnessMode whichMode) {
	std::string PrintString = ";";
	switch (whichMode) {
	//kOff, kOn, kBreathing, kBlinking, kDashed, kRollIn, kRollOut, kRolling
	case kOn:
		PrintString = "On";
		break;
	case kBreathing:
		PrintString = "Breathing";
		break;
	case kBlinking:
		PrintString = "Blinking";
		break;
	case kDashed:
		PrintString = "Dashed";
		break;
	case kRollIn:
		PrintString = "RollIn";
		break;
	case kRollOut:
		PrintString = "RollOut";
		break;
	case kRolling:
		PrintString = "Rolling";
		break;
	default:
		PrintString = "Off";
		break;
	}
	serialPort->Write(PrintString);

}

void Arduino::GetCameraData(bool& isFarLeft, bool& isAligned, bool& isTooFar) {
	static bool farLeft = false;
	static bool aligned = true;
	static bool badDistance = true;

	std::string CameraData = GetCameraData();
	if (strncmp(CameraData.c_str(), "Camera", 6) == 0
			&& CameraData[12] == ';') {
		if (CameraData[7] == 'L')
			farLeft = true;
		else
			farLeft = false;

		if (CameraData[9] == 'N')
			badDistance = false;
		else
			badDistance = true;

		if (CameraData[7] == 'G')
			aligned = true;
		else
			aligned = false;

	}

	isFarLeft = farLeft;
	isAligned = aligned;
	isTooFar = badDistance;
}
