
const unsigned long HeartRateSecs = 1;

void Translator (const char * input, Mode& mode, State& state, bool& isBatteryLow){
  static Mode localMode = kDemo;
  static State localState = kBreathing;
  static bool localBatteryLow = false;
  static unsigned long lastHeartbeat = 0;

  if (strcmp(input, "RedTeam") == 0)
    localMode = kRedTeam;
  else if (strcmp(input, "BlueTeam") == 0)
    localMode = kBlueTeam;
  else if (strcmp(input, "Demo") == 0)
    localMode = kDemo;
  else if (strcmp(input, "Disabled") == 0)
    localState = kBreathing;
  else if (strcmp(input, "Solid") == 0)
    localState = kSolid;
  else if (strcmp(input, "SlowBlink") == 0)
    localState = kSlowBlink;
  else if (strcmp(input, "MediumBlink") == 0)
    localState = kBlink;
  else if (strcmp(input, "Off") == 0)
    localState = kOff;
  else if (strcmp(input, "LowBattery") == 0)
    localBatteryLow = true;
  else if (strcmp(input, "GoodBattery") == 0)
    localBatteryLow = false;
  else if (strcmp(input, "Heartbeat") == 0)
    lastHeartbeat = millis();
  else  {
    localMode = kError;
    localState = kSolid;
    Serial.print("Don't understand: '");
    Serial.print(input);
    Serial.println("'");
    Serial.print(";");
  }
  
  if (millis() - lastHeartbeat > HeartRateSecs * 2000){
    localMode = kError;
  }

 

  mode = localMode;
  state = localState;
  isBatteryLow = localBatteryLow;
}





