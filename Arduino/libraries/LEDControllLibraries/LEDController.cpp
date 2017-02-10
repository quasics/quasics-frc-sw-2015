#ifndef LEDSERIALCONTROLLER_H_
#define LEDSERIALCONTROLLER_H_

#include "LEDController.h"

LEDController::LEDController(int redIn, int greenIn, int blueIn, int whiteIn, int maxRed,
		int maxGreen, int maxBlue) {
	pinMode(redIn, OUTPUT);
	pinMode(greenIn, OUTPUT);
	pinMode(blueIn, OUTPUT);
  pinMode(whiteIn, OUTPUT);
	redPin = redIn;
	greenPin = greenIn;
	bluePin = blueIn;
  whitePin = whiteIn;

	redMax = maxRed;
	greenMax = maxGreen;
	blueMax = maxBlue;

	redValue = 0;
	greenValue = 0;
	blueValue = 0;
  whiteValue = 0;
	brightnessPercent = 1;
}

//Credit to https://github.com/ratkins/RGBConverter
void LEDController::SetHSV(double h, double s, double v) {
	double r, g, b;

	int i = int(h * 6);
	double f = h * 6 - i;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f) * s);

	switch (i % 6) {
	case 0:
		r = v, g = t, b = p;
		break;
	case 1:
		r = q, g = v, b = p;
		break;
	case 2:
		r = p, g = v, b = t;
		break;
	case 3:
		r = p, g = q, b = v;
		break;
	case 4:
		r = t, g = p, b = v;
		break;
	case 5:
		r = v, g = p, b = q;
		break;
	}

	SetRed(r * 255);
	SetGreen(g * 255);
	SetBlue(b * 255);
}

void LEDController::SetRed(int value) {
	redValue = value;
	analogWrite(redPin, int((redValue/255) * redMax * brightnessPercent));
}

void LEDController::SetGreen(int value) {
	greenValue = value;

	analogWrite(greenPin, int((greenValue/255) * greenMax * brightnessPercent));
}

void LEDController::SetBlue(int value) {
	blueValue = value;

	analogWrite(bluePin, int((blueValue/255) * blueMax * brightnessPercent));
}

void LEDController::SetWhite(int value){
  whiteValue = value;

  analogWrite(whitePin, int(whiteValue * brightnessPercent));
}

void LEDController::SetBrightness(float percent) {
	brightnessPercent = max(min(float(percent), 1), 0);

	analogWrite(redPin, int((redValue/255) * redMax * brightnessPercent));
	analogWrite(greenPin, int((greenValue/255) * greenMax * brightnessPercent));
	analogWrite(bluePin, int((blueValue/255) * blueMax * brightnessPercent));
  analogWrite(whitePin, int((whiteValue) * brightnessPercent));

}

int LEDController::GetRed() {
	return redValue;
}

int LEDController::GetGreen() {
	return greenValue;
}

int LEDController::GetBlue() {
	return blueValue;
}

int LEDController::GetWhite(){
  return whiteValue;
}

int LEDController::GetBrightness() {
	return brightnessPercent;
}
#endif

