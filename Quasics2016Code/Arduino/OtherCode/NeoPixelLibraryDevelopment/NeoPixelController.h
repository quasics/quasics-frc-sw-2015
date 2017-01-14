#ifndef NEOPIXELCONTROLLER_H_
#define NEOPIXELCONTROLLER_H_

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>

class NeoPixelController {
public:
	enum Mode {
		kDefaultMode, kCycle, kRed, kGreen, kBlue, kWhite, kRainbow, kGreenToWhite, kWhiteToRed, kWhiteToGreen, kWhiteToBlue
	};
	enum Pattern {
		kDefaultPattern, kRollIn, kRollOut, kRolling, kOff, kOn
	};
	enum Evolution {
		kNoEvolution = 0, kRollingCycle = 1
	};

	NeoPixelController(uint32_t stripLength, uint32_t pin,
			uint32_t brightnessInit);	//Constructor
	~NeoPixelController();	//Destructor

	//Basic Control Mechanisms
	void SetStripColor(uint32_t red, uint32_t green, uint32_t blue,
			uint32_t white);	//Set Whole Strip RGBW (0-255)
	void SetStripColor(float hue, float saturation, float value);	//Set Whole Strip HSV (0-1)

	void SetRangeColor(uint32_t first, uint32_t last, uint32_t red,
			uint32_t green, uint32_t blue, uint32_t white);	//Set RGBW (0-255) from first to last
	void SetRangeColor(uint32_t first, uint32_t last, float hue,
			float saturation, float value);	//Set HSV (0-1) from first to last

	void SetStripBrightness(float brightnessLevel);	//Adjust Brightness (Max Valance) (0-1)
	void SetRangeBrightness(uint32_t first, uint32_t last,
			float brightnessLevel);	//Adjust Brightness (Max Valance) (0-1) from first to last

	void Show();	//display current colors and values

	//Pattern Storage
	void SetMode(Mode whichMode);	//Control color patterns (solid and dynamic)
	void SetPattern(Pattern whichPattern);//Control lighting dynamics (full strip or partial)
	void SetEvolution(Evolution whichEvolution);//Coordinate Modes and Patterns

	//Process
	void Process();	//Carry out Mode and Pattern instructions
	void Evolve();	//Carry out Evolutions (handles Mode and Pattern setting)

	//Return Data
	float GetPixelBrightness(uint32_t pixel);//Returns the specified pixel's brightness multiplier
	uint32_t GetPixelColor(uint32_t pixel);	//Returns the combined (32 bit) color value on specified pixel
	uint32_t GetLength();	//Returns Strand Length

	bool InPattern();	//Returns true if carrying out a brightness change
	bool InMode();	//Returns true if carrying out a color change
	bool InEvolution();	//Returns true if carrying out an evolution

	Pattern GetPattern();	//Returns current pattern setting
	Mode GetMode();	//Returns current color setting
	Evolution GetEvolution();	//returns current evolution

private:
	uint32_t lastPixel;
	uint32_t halfPixel;

	uint32_t modeIteration;

	uint32_t* colorData;
	uint32_t* brightnessData;

	bool inPattern;
	bool inMode;
	bool inEvolution;

	Mode mode;
	Pattern pattern;
	Evolution evolution;

	Adafruit_NeoPixel* strip;
};

#endif
