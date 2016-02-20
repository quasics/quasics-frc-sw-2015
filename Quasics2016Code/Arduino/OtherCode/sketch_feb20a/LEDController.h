#ifndef LEDCONTROLLER_H_
#define LEDCONTROLLER_H_

class LEDController {
public:
  LEDController (int redIn, int greenIn, int blueIn);
  
  void SetRed (int value);
  void SetGreen (int value);
  void SetBlue (int value);
  void SetBrightness (float percent);
  
  //Credit to https://github.com/ratkins/RGBConverter
  void SetHSV(double h, double s, double l);

  int GetRed ();
  int GetGreen ();
  int GetBlue ();
  int GetBrightness ();

private:

  int redPin;
  int greenPin;
  int bluePin;

  float redValue;
  float greenValue;
  float blueValue;
  float brightnessPercent;
};

#endif

