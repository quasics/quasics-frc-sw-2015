/*
 * NeoPixelPatternManager.h
 *
 *  Created on: Feb 22, 2017
 *      Author: Yogna
 */

#ifndef NEOPIXELPATTERNMANAGER_H_
#define NEOPIXELPATTERNMANAGER_H_

#include "NeoPixelController.h"

class NeoPixelPatternManager {
public:
	NeoPixelPatternManager(uint32_t stripLength, uint32_t pin,
			uint32_t brightnessInit, bool isRGBW = false);
	virtual ~NeoPixelPatternManager();

//--------------------------------------------------------<ENUMS>-----------------------------------------------------------------------------------------
	enum Mode {
		kDefaultMode,
		kCycle,
		kRed,
		kGreen,
		kBlue,
		kWhite,
		kRainbow,
		kGreenToWhite,
		kWhiteToRed,
		kWhiteToGreen,
		kWhiteToBlue
	};
	enum Pattern {
		kDefaultPattern, kRollIn, kRollOut, kRolling, kOff, kOn
	};
	enum Evolution {
		kNoEvolution = 0, kRollingCycle = 1
	};
//---------------------------------------------------<Control Functions>----------------------------------------------------------------------------------
//---------------------------------------------------<Pattern Storage>------------------------------------------------------------------------------------
	void SetMode(Mode whichMode);	//Control color patterns (solid and dynamic)
	void SetPattern(Pattern whichPattern);//Control lighting dynamics (full strip or partial)
	void SetEvolution(Evolution whichEvolution);//Coordinate Modes and Patterns

///----------------------------------------------------<Process>------------------------------------------------------------------------------------------
	void Process();	//Carry out Mode and Pattern instructions
	void Evolve();	//Carry out Evolutions (handles Mode and Pattern setting)

//----------------------------------------------------<Data Return>---------------------------------------------------------------------------------------
	bool InPattern();	//Returns true if carrying out a brightness change
	bool InMode();	//Returns true if carrying out a color change
	bool InEvolution();	//Returns true if carrying out an evolution

	Pattern GetPattern();	//Returns current pattern setting
	Mode GetMode();	//Returns current color setting
	Evolution GetEvolution();	//returns current evolution

private:
	void RangeHSVTranslation(uint32_t firstPixel, uint32_t finalPixel,
			float finalHue, float finalSaturation, float finalValue,
			uint32_t numberOfIterations);
	bool RangeHSVTranslation();	//Continues the previously setTranslation
	void StripHSVTranslation (float finalHue, float finalSaturation, float finalValue,
			uint32_t numberOfIterations);
	bool StripHSVTranslation ();

	Mode mode;
	Pattern pattern;
	Evolution evolution;

	uint32_t lastPixel;
	uint32_t halfPixel;

	uint32_t modeIteration;

	bool inPattern;
	bool inMode;
	bool inEvolution;

	NeoPixelController* strip;

	uint32_t HSVTranslateFirstPixel;
	uint32_t HSVTranslateLastPixel;
	float HSVTranslateHue;
	float HSVTranslateSaturation;
	float HSVTranslateValue;
	uint32_t HSVTranslateIterations;
	uint32_t HSVTranslateCounter;

	const int hsvTranslationSteps = 50;
};

#endif /* NEOPIXELPATTERNMANAGER_H_ */
