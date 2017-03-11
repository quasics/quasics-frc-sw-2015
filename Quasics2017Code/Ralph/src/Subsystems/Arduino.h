#ifndef Arduino_H
#define Arduino_H

#include "WPILib.h"

class Arduino: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Arduino();
	void InitDefaultCommand();

	enum ColorMode {
		kNone,
		kRed,
		kGreen,
		kBlue,
		kWhite,
		kRainbow,
		kOldCycle,
		kYellow,
		kQuasics,
		kBrown
	};

	enum BrightnessMode {
		kOff, kOn, kBreathing, kBlinking, kDashed, kRollIn, kRollOut, kRolling
	};

	void SetLEDColor(ColorMode whichColor);
	void SetBrightnessMode(BrightnessMode whichMode);
	void GetCameraData(bool& isFarLeft, bool& isAligned, bool& isTooFar);

private:
	std::shared_ptr<SerialPort> serialPort;

	std::string GetCameraData();
};

#endif  // Arduino_H
