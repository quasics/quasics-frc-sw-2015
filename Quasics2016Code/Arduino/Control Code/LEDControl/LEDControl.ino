#include "Includes.h"

LEDController* lightControl = NULL;
String serialIn = "";
Mode activeMode = kDemo;
State activeState = kBreathing;
bool activeBatteryLow = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lightControl = new LEDController (RedPin, GreenPin, BluePin);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0)
  {
    char c = char(Serial.read());
    if (c != ';') {
      serialIn += c;
    } 
    else {
      Translator (serialIn.c_str(), activeMode, activeState, activeBatteryLow);
      Serial.print("activeMode = ");
      Serial.println(activeMode);
      Serial.print(";");
      serialIn = "";
    }
    
  }
  
  
  SetMode (activeMode);
  SetState (activeState);
  SetBatteryLow (activeBatteryLow);
}









