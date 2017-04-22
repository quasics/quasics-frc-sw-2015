#ifndef NEOPIXELCONTROLLER_H_
#define NEOPIXELCONTROLLER_H_

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>

class NeoPixelController {
public:
	NeoPixelController(uint32_t stripLength, uint32_t pin,
			uint32_t brightnessInit, bool isRGBW = false);	//Constructor
	~NeoPixelController();	//Destructor

	//Basic Control Mechanisms
	void SetStripColor(uint32_t red, uint32_t green, uint32_t blue);//Set Whole Strip RGBW (0-255)
	void SetStripHSV(float hue, float saturation, float value);	//Set Whole Strip HSV (0-1)

	void SetRangeColor(uint32_t first, uint32_t last, uint32_t red,
			uint32_t green, uint32_t blue);	//Set RGBW (0-255) from first to last
	void SetRangeHSV(uint32_t first, uint32_t last, float hue, float saturation,
			float value);	//Set HSV (0-1) from first to last

	void SetStripBrightness(float brightnessLevel);	//Adjust Brightness (Max Valance) (0-1)
	void SetRangeBrightness(uint32_t first, uint32_t last,
			float brightnessLevel);	//Adjust Brightness (Max Valance) (0-1) from first to last

	void AdvanceStripSetting(int numberOfPlaces);

	void Show();	//display current colors and values

	//Return Data
	float GetPixelBrightness(uint32_t pixel);//Returns the specified pixel's brightness multiplier
	uint32_t GetPixelColor(uint32_t pixel);	//Returns the combined (32 bit) color value on specified pixel
	uint32_t GetLength();	//Returns Strand Length

	void HSVToRGB(float hue, float saturation, float value, uint32_t& r,
			uint32_t& g, uint32_t& b);
	void RGBToHSV(uint32_t red, uint32_t green, uint32_t blue, float& hue,
			float& saturation, float& value);
	void ColorCodeToRGB(uint32_t code, uint32_t& red, uint32_t& green,
			uint32_t& blue);
	void ColorCodeToHSV(uint32_t code, float& hue, float& saturation,
			float& value);
private:

	uint32_t lastPixel;
	uint32_t halfPixel;

	uint32_t* colorData;
	float* brightnessData;

	bool isGRBW;

	Adafruit_NeoPixel* strip;
};

#endif
