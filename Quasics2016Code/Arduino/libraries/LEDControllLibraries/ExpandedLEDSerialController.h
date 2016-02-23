#ifndef LEDSERIALCONTROLLER_H_
#define LEDSERIALCONTROLLER_H_

#include "LEDController.h"
#include "Arduino.h"

class ExpandedLEDSerialController: public LEDController {
  public:
    ExpandedLEDSerialController (unsigned int redPin, unsigned int greenPin, unsigned int bluePin, unsigned int baseLoopSeconds);
    void LEDSerialProcess ();

  private:
    enum Mode {
      kRed, kBlue, kGreen, kYellow, kMagenta, kCyan, kOrange, kWhite, kPurple, kCycle, kRainbow, kError
    };
    enum State {
      kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kOff
    };
    
    bool isLowBatteryOverride;
    String serialIn;
    Mode activeMode;
    State activeState;
    unsigned long loopSeconds;


    void SetMode (Mode mode = kError);
    void SetState (State state = kBreathing);
    void Translator (const char * input, Mode& mode, State& state);
};

#endif
