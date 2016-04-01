#ifndef LEDCONTROLLER_H_
#define LEDCONTROLLER_H_

class LEDController {
public:
  LEDController(unsigned int redIn, unsigned int greenIn, unsigned int blueIn, unsigned int whiteIn, unsigned int maxRed = 100,
      unsigned int maxGreen = 127, unsigned int maxBlue = 255);

  void SetRed(unsigned int value = 0);
  void SetGreen(unsigned int value = 0);
  void SetBlue(unsigned int value = 0);
  void SetWhite(unsigned int value = 0);
  void SetBrightness(float percent =0);

  //Credit to https://github.com/ratkins/RGBConverter
  void SetHSV(double h, double s, double l);

  unsigned int GetRed();
  unsigned int GetGreen();
  unsigned int GetBlue();
  unsigned int GetWhite ();
  float GetBrightness();

private:

  unsigned int redPin;
  unsigned int greenPin;
  unsigned int bluePin;
  unsigned int whitePin;

  unsigned int redMax;
  unsigned int greenMax;
  unsigned int blueMax;

  unsigned int redValue;
  unsigned int greenValue;
  unsigned int blueValue;
  unsigned int whiteValue;

  float brightnessPercent;

};

#endif

