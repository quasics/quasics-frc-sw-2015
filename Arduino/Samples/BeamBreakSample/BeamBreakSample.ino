/* 
  IR Breakbeam sensor demo!

  Based on code from: https://learn.adafruit.com/ir-breakbeam-sensors?view=all
*/

#define LEDPIN 13
  // Pin 13: Arduino has an LED connected on pin 13
  // Pin 11: Teensy 2.0 has the LED on pin 11
  // Pin  6: Teensy++ 2.0 has the LED on pin 6
  // Pin 13: Teensy 3.0 has the LED on pin 13

#define SENSORPIN 4

// variables will change:
int sensorState = 0, lastState=-1;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);      
  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT);     
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  
  Serial.begin(115200);
  Serial.println("Beginning execution");
}

void loop(){
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);

  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (sensorState == LOW) {     
    // turn LED on:
    digitalWrite(LEDPIN, HIGH);  
  } 
  else {
    // turn LED off:
    digitalWrite(LEDPIN, LOW); 
  }

  if (sensorState != lastState) {
    if (sensorState == LOW) {
      Serial.println("Sensor state: Broken");
    } else {
      Serial.println("Sensor state: Unbroken");
    }
  }
  lastState = sensorState;
}
