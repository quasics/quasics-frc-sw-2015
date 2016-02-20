#include "LEDController.h"
#include "Arduino.h"

class LEDSerialController {
  public:
    LEDSerialController (unsigned int redPin, unsigned int greenPin, unsigned int bluePin );
    void LEDSerialProcess ();

  private:
    enum Mode {
      kRedTeam, kBlueTeam, kDemo, kError
    };
    enum State {
      kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff
    };

    LEDController* lightControl;
    bool isLowBatteryOverride;
    const unsigned long HeartRateSecs = 1;
    String serialIn;
    Mode activeMode;
    State activeState;
    bool activeBatteryLow;


    void SetMode (Mode mode = kError);
    void SetState (State state = kBreathing);
    void SetBatteryLow (bool isLow = false);
    void Translator (const char * input, Mode& mode, State& state, bool& isBatteryLow);
};
