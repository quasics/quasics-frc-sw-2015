#ifndef ArduinoController_H
#define ArduinoController_H

#include <Commands/Subsystem.h>

class ArduinoController: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	enum ColorMode {
		kNone,
		kRed,
		kGreen,
		kBlue,
		kWhite,
		kRainbow,
		kRainbowReverse,
		kOldCycle,
		kYellow,
		kQuasics,
		kBrown,
		kPurple
	};
	enum BrightnessMode {
		kOff,
		kOn,
		kBreathing,
		kBlinking,
		kDashed,
		kRollIn,
		kRollOut,
		kRolling,
		kSnakeIn,
		kSnakeOut,
		kSnake
	};

	ArduinoController();
	void SetLightColor(ColorMode colorMode);
	void SetLightDynamic(BrightnessMode brightnessMode);

	ColorMode GetColorMode ();
	BrightnessMode GetBrightnessMode ();

	void InitDefaultCommand();
private:
	ColorMode currentColor;
	BrightnessMode currentBrightness;
};

#endif  // ArduinoController_H
