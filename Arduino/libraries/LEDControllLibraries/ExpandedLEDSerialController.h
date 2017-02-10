#ifndef LEDSERIALCONTROLLER_H_
#define LEDSERIALCONTROLLER_H_

#include "LEDController.h"
#include "Arduino.h"

class ExpandedLEDSerialController: public LEDController {
  public:
	enum Mode {
      kRed, kBlue, kGreen, kYellow, kMagenta, kCyan, kOrange, kWhite, kPurple, kCycle, kRainbow, kError
    };
    enum State {
      kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kOff
    };

    ExpandedLEDSerialController (unsigned int redPin, unsigned int greenPin, unsigned int bluePin, unsigned int whitePin, unsigned int baseLoopSeconds);
    void LEDSerialProcess ();
    void SetMode (Mode mode = kError);
    void SetState (State state = kBreathing);

  protected:    
    bool isLowBatteryOverride;
    String serialIn;
    Mode activeMode;
    State activeState;
    unsigned long loopSeconds;

    void Translator (const char * input, Mode& mode, State& state);
};

#endif
