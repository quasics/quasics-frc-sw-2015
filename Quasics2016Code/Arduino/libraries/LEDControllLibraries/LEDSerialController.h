#include "LEDController.h"
#include "Arduino.h"

class LEDSerialController: public LEDController {
  public:
    LEDSerialController (unsigned int redPin, unsigned int greenPin, unsigned int bluePin, unsigned int whitePin, unsigned long heartRateSeconds);
    void LEDSerialProcess ();

  private:
    enum Mode {
      kRedTeam, kBlueTeam, kDemo, kError
    };
    enum State {
      kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff
    };
    
    bool isLowBatteryOverride;
    unsigned long HeartRateSecs;
    String serialIn;
    Mode activeMode;
    State activeState;
    bool activeBatteryLow;
    unsigned long lastHeartbeat;


    void SetMode (Mode mode = kError);
    void SetState (State state = kBreathing);
    void SetBatteryLow (bool isLow = false);
    void Translator (const char * input, Mode& mode, State& state, bool& isBatteryLow);
};
