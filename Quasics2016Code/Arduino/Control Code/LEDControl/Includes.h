#ifndef INCLUDES_H_
#define INCLUDES_H_

#include "IOMap.h"
#include <LEDController.h>


enum Mode{
  kRedTeam, kBlueTeam, kDemo, kError
};
enum State {
  kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff
};
void SetMode (Mode mode = kError);
void SetState (State state = kBreathing);
void SetBatteryLow (bool isLow = false);
void Translator (const char * input, Mode& mode, State& state, bool& isBatteryLow); 

#endif
