#ifndef LEDCONTROLLER_H_
#define LEDCONTROLLER_H_

#include "Arduino.h"

class LEDController {
public:
	LEDController(int redIn, int greenIn, int blueIn, int whiteIn, int maxRed = 100,
			int maxGreen = 127, int maxBlue = 255);

	void SetRed(int value = 0);
	void SetGreen(int value = 0);
	void SetBlue(int value = 0);
  void SetWhite(int value = 0);
	void SetBrightness(float percent = 1);

	//Credit to https://github.com/ratkins/RGBConverter
	void SetHSV(double h, double s, double l);

	int GetRed();
	int GetGreen();
	int GetBlue();
  int GetWhite();
	int GetBrightness();

private:

	int redPin;
	int greenPin;
	int bluePin;
  int whitePin;

	int redMax;
	int greenMax;
	int blueMax;

	float redValue;
	float greenValue;
	float blueValue;
  float whiteValue;
	float brightnessPercent;
};

#endif

