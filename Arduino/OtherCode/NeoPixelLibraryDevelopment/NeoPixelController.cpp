#include "NeoPixelController.h"

//--------------------------------------Constructor and Destructor---------------------------------------
NeoPixelController::NeoPixelController(uint32_t stripLength, uint32_t pin,
		uint32_t brightnessInit) {
	//Neopixel object Setup
	strip = new Adafruit_NeoPixel(stripLength, pin, NEO_GRBW + NEO_KHZ800);
	strip->begin();
	strip->setBrightness(brightnessInit);
	strip->show();

	//Color & brightness aray setup
	colorData = new uint32_t[stripLength];
	brightnessData = new uint32_t[stripLength];
	for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
		colorData[pixel] = strip->Color(0, 0, 0, 0);
	}
	for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
		brightnessData[pixel] = 1;
	}

	//Store the physical parameters about the strip
	lastPixel = stripLength - 1;
	halfPixel = uint32_t(lastPixel / 2 + .5);

	//Set Iteration Clocks
	modeIteration = 0;

	//Set the default states for evolutions, modes, and paterns
	mode = kDefaultMode;
	pattern = kDefaultPattern;
	evolution = kNoEvolution;

	//
	inMode = false;
	inPattern = false;
	inEvolution = false;
}

NeoPixelController::~NeoPixelController() {
	delete[] colorData;
	colorData = NULL;
	delete strip;
	strip = NULL;
	delete[] brightnessData;
	brightnessData = NULL;
}

//--------------------------------------Processes--------------------------------------------------------
void NeoPixelController::Process() {
	switch (mode) {
	case kCycle:
		for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
			SetRangeColor(pixel, pixel,
					float(
							int(modeIteration + 360 / (lastPixel + 1) * pixel)
									% 360) / 360, 1, 1);
		}
		modeIteration = ((modeIteration) % 360) + 1;
		break;
	case kRed:
		SetStripColor(255, 0, 0, 0);
		break;
	case kGreen:
		SetStripColor(0, 255, 0, 0);
		break;
	case kBlue:
		SetStripColor(0, 0, 255, 0);
		break;
	case kRainbow:
		for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
			SetRangeColor(pixel, pixel,
					float(int(360 / (lastPixel + 1) * pixel) % 360) / 360, 1,
					brightnessData[pixel]);
		}
		break;
	case kGreenToWhite:
		for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
			SetRangeColor(pixel, pixel, 1 / 3, 1 - modeIteration / 1000, 1);
		}
		modeIteration = ((modeIteration) % 1000) + 1;
		if (modeIteration >= 1000) {

		}
		break;
	default:
		SetStripColor(0, 0, 0, 255);
	}

	switch (pattern) {
	case kRollIn:
		static uint32_t pixelsFilled = 1;
		static uint32_t iteration = 0;
		SetRangeBrightness(0, GetLength() - pixelsFilled, 0);
		SetRangeBrightness(iteration, iteration, 1);

		if (iteration >= GetLength() - pixelsFilled) {
			iteration = 0;
			if (pixelsFilled >= GetLength()) {
				inPattern = false;
				pattern = kDefaultPattern;
				pixelsFilled = 1;
			} else {
				pixelsFilled++;
			}
		} else {
			iteration++;
		}
		break;
	case kRollOut:
		SetRangeBrightness(0, GetLength() - pixelsFilled, 1);
		SetRangeBrightness(iteration, iteration, 0);

		if (iteration >= GetLength() - pixelsFilled) {
			iteration = 0;
			if (pixelsFilled >= GetLength()) {
				inPattern = false;
				pattern = kDefaultPattern;
				pixelsFilled = 1;
			} else {
				pixelsFilled++;
			}
		} else {
			iteration++;
		}
		break;
	case kRolling:
		static uint32_t stage = 0;
		if (stage == 0) {
			static uint32_t pixelsFilled = 1;
			static uint32_t iteration = 0;
			SetRangeBrightness(0, GetLength() - pixelsFilled, 0);
			SetRangeBrightness(iteration, iteration, 1);

			if (iteration >= GetLength() - pixelsFilled) {
				iteration = 0;
				if (pixelsFilled >= GetLength()) {
					stage = 1;
					pixelsFilled = 1;
				} else {
					pixelsFilled++;
				}
			} else {
				iteration++;
			}
		} else if (stage == 1) {
			static uint32_t pixelsFilled = 1;
			static uint32_t iteration = 0;
			SetRangeBrightness(0, GetLength() - pixelsFilled, 1);
			SetRangeBrightness(iteration, iteration, 0);

			if (iteration >= GetLength() - pixelsFilled) {
				iteration = 0;
				if (pixelsFilled >= GetLength()) {
					stage = 0;
					pixelsFilled = 1;
				} else {
					pixelsFilled++;
				}
			} else {
				iteration++;
			}
		}
		break;
	case kOff:
		for (uint32_t pixel = 0; pixel >= lastPixel; pixel++) {
			brightnessData[pixel] = 0;
		}
		inPattern = false;
		break;
	default:
		for (uint32_t pixel = 0; pixel >= lastPixel; pixel++) {
			brightnessData[pixel] = 1;
		}
		inPattern = false;
		break;
	}

	Show();
}

void NeoPixelController::Evolve() {
	switch (evolution) {
	case kRollingCycle:
		static uint32_t stage = 0;

		static uint32_t startTime = 0;
		switch (stage) {
		case 0:
			static bool initialized = false;
			if (!initialized) {
				SetMode(NeoPixelController::kRainbow);
				SetPattern(NeoPixelController::kRollIn);
				initialized = true;
			}
			delay(25);
			if (!inPattern) {
				stage = 1;
				initialized = false;
			}
			break;
		case 1:
			if (!initialized) {
				SetMode(NeoPixelController::kCycle);
				SetPattern(NeoPixelController::kOn);
				initialized = true;
				startTime = millis();
			}
			if (millis() - startTime >= 6775) {
				stage = 2;
				initialized = false;
			}
			break;
		case 2:
			if (!initialized) {
				SetMode(NeoPixelController::kRainbow);
				SetPattern(NeoPixelController::kRollOut);
				initialized = true;
			}
			delay(25);
			if (!inPattern) {
				stage = 0;
				initialized = false;
			}
			break;
		default:
			stage = 0;
			initialized = false;
		}
		break;
	default:
		break;
	}

	Process();
}

//--------------------------------------Basic Control Mechanisms-----------------------------------------
void NeoPixelController::SetStripColor(uint32_t red, uint32_t green,
		uint32_t blue, uint32_t white) {
	for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
		uint32_t combined = strip->Color(red * brightnessData[pixel],
				green * brightnessData[pixel], blue * brightnessData[pixel],
				white * brightnessData[pixel]);
		strip->setPixelColor(pixel, combined);
		colorData[pixel] = combined;
	}
}

void NeoPixelController::SetStripColor(float h, float s, float v) {
	if (s != 0) {
		double r = 0;
		double g = 0;
		double b = 0;

		int i = int(h * 6);
		double f = h * 6 - i;
		double p = v * (1 - s);
		double q = v * (1 - f * s);
		double t = v * (1 - (1 - f) * s);

		switch (i % 6) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		case 5:
			r = v;
			g = p;
			b = q;
			break;
		}
		for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel], 0);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	} else {
		for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
			uint32_t combined = strip->Color(0, 0, 0, v * 255);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	}
}

void NeoPixelController::SetRangeColor(uint32_t startPixel, uint32_t endPixel,
		uint32_t r, uint32_t g, uint32_t b, uint32_t w) {
	if ((endPixel <= lastPixel)) {
		for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel],
					w * 255 * brightnessData[pixel]);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	} else {
		for (uint32_t pixel = startPixel; pixel <= lastPixel; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel],
					w * 255 * brightnessData[pixel]);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
		for (uint32_t pixel = 0; pixel <= endPixel % lastPixel - 1; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel],
					w * 255 * brightnessData[pixel]);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	}
}

void NeoPixelController::SetRangeColor(uint32_t startPixel, uint32_t endPixel,
		float h, float s, float v) {

	double r = 0;
	double g = 0;
	double b = 0;

	int i = int(h * 6);
	double f = h * 6 - i;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f) * s);

	switch (i % 6) {
	case 0:
		r = v;
		g = t;
		b = p;
		break;
	case 1:
		r = q;
		g = v;
		b = p;
		break;
	case 2:
		r = p;
		g = v;
		b = t;
		break;
	case 3:
		r = p;
		g = q;
		b = v;
		break;
	case 4:
		r = t;
		g = p;
		b = v;
		break;
	case 5:
		r = v;
		g = p;
		b = q;
		break;
	}

	if ((endPixel <= lastPixel)) {
		for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel], 0);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	} else {
		for (uint32_t pixel = startPixel; pixel <= lastPixel; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel], 0);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
		for (uint32_t pixel = 0; pixel <= endPixel % lastPixel - 1; pixel++) {
			uint32_t combined = strip->Color(r * 255 * brightnessData[pixel],
					g * 255 * brightnessData[pixel],
					b * 255 * brightnessData[pixel], 0);
			strip->setPixelColor(pixel, combined);
			colorData[pixel] = combined;
		}
	}
}

void NeoPixelController::SetStripBrightness(float brightnessLevel) {
	for (uint32_t pixel = 0; pixel <= lastPixel; pixel++)
		brightnessData[pixel] = brightnessLevel;
}

void NeoPixelController::SetRangeBrightness(uint32_t first, uint32_t last,
		float brightnessLevel) {
	for (uint32_t pixel = first; pixel <= last; pixel++)
		brightnessData[pixel] = brightnessLevel;
}

void NeoPixelController::Show() {
	strip->show();
}

//--------------------------------------Pattern storage--------------------------------------------------
void NeoPixelController::SetPattern(Pattern whichPattern) {
	pattern = whichPattern;
	inPattern = true;
}

void NeoPixelController::SetMode(Mode whichMode) {
	mode = whichMode;
	modeIteration = 0;
}

void NeoPixelController::SetEvolution(Evolution whichEvolution) {
	evolution = whichEvolution;
}

//--------------------------------Data Return Functions--------------------------------------------------
//Get Physical Data about the strip
uint32_t NeoPixelController::GetPixelColor(uint32_t pixel) {
	return colorData[pixel];
}

float NeoPixelController::GetPixelBrightness(uint32_t pixel) {
	return brightnessData[pixel];
}

uint32_t NeoPixelController::GetLength() {
	return lastPixel + 1;
}

//Return Dynamic activity
bool NeoPixelController::InPattern() {
	return inPattern;
}

bool NeoPixelController::InMode() {
	return inMode;
}

bool NeoPixelController::InEvolution() {
	return inEvolution;
}

//Return current dynamic preset
NeoPixelController::Pattern NeoPixelController::GetPattern() {
	return pattern;
}

NeoPixelController::Mode NeoPixelController::GetMode() {
	return mode;
}

NeoPixelController::Evolution NeoPixelController::GetEvolution() {
	return evolution;
}
