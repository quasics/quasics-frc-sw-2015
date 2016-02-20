bool isLowBatteryOverride = false;

void SetMode (Mode mode){
  if (!isLowBatteryOverride){
    switch (mode){
    case kRedTeam:
      lightControl->SetRed (255);
      lightControl->SetGreen (0);
      lightControl->SetBlue (0);
      break;
    case kBlueTeam:
      lightControl->SetRed (0);
      lightControl->SetGreen (0);
      lightControl->SetBlue (255);
      break;
    case kDemo:
      lightControl->SetRed (0);
      lightControl->SetGreen (255);
      lightControl->SetBlue (0);
      break;
    default:
      lightControl->SetRed (255);
      lightControl->SetGreen (255);
      lightControl->SetBlue (0);
      break;
    }
  }
}

//kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff

void SetState (State state){
  switch (state){
  case kBreathing:
    {
      const float loopDurration = 4;
      const float halfDurration = loopDurration/2;

      float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;
      float timeHalf = float(millis() % int(halfDurration * 1000)) / 1000;

      if (timeLoop <= halfDurration)
        lightControl->SetBrightness((timeHalf * timeHalf)/(halfDurration * halfDurration));
      else
        lightControl->SetBrightness(((halfDurration * halfDurration) - (timeHalf * timeHalf)) / 
        (halfDurration * halfDurration));
    }
    break;

  case kBlink:
    {
      const float loopDurration = 2;
      const float halfDurration = loopDurration/2;

      float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;

      if (timeLoop <= halfDurration)
        lightControl->SetBrightness(1);
      else
        lightControl->SetBrightness(0);
    }
    break;

  case kSlowBlink:
    {
      const float loopDurration = 3;
      const float halfDurration = loopDurration/2;

      float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;

      if (timeLoop <= halfDurration)
        lightControl->SetBrightness(1);
      else
        lightControl->SetBrightness(0);
    }
    break;

  case kSolid:
    lightControl->SetBrightness(1);
    break;

  case kOff:
    lightControl->SetBrightness(0);
    break;

  default: //and quick blink are the same thing
    {
      const float loopDurration = 1;
      const float halfDurration = loopDurration/2;

      float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;


      if (timeLoop <= halfDurration)
        lightControl->SetBrightness(1);
      else
        lightControl->SetBrightness(0);
    }
    break;
  }
}

void SetBatteryLow (bool isLow){
  if (isLow && millis() % 5000 > 4000){
    lightControl->SetRed(255);
    lightControl->SetGreen(255);
    lightControl->SetBlue(0);
    isLowBatteryOverride = true;
  }
  else{
    isLowBatteryOverride = false;
  }
}

