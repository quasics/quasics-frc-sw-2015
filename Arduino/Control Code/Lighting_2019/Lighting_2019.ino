#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      25

int delayval = 43; // delay for half a second

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);
void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  Serial.begin(115200);
}

void showOneCycle(uint32_t color) {
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,150,0 up to 255,255,255
    pixels.setPixelColor(i, color);
    pixels.show();
    delay(delayval); // Delay for a period of time (150 miliseconds).

  }

    for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,150,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.show();
    delay(delayval); // Delay for a period of time (150 miliseconds).

  }
}

String text;    // buffer used to hold input from the user, prior to executing as a command.
uint32_t color = pixels.Color(0,150,0);

void loop() {
  //   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255

//  showOneCycle(pixels.Color(0,150,0)); // Moderately bright green color.
//  showOneCycle(pixels.Color(150,0,0)); // Moderately bright red color.
//  showOneCycle(pixels.Color(0,0,150)); // Moderately bright blue color.

  // See if we have a command to be executed.
  String command;
  while(Serial.available() > 0) {
    char nextChar = Serial.read();
    if (nextChar != '\n' && nextChar != '\r') {
      text += nextChar;
    } else {
      command = text;
      text = "";
    }
  }

  // If we have a command, do something with it.
  if (command != "") {
    if (command == "red")  {
     color=(pixels.Color(150,0,0)); // Moderately bright red color.
    } else if (command == "blue") {
      color=(pixels.Color(0,0,150)); // Moderately bright blue color.
    } else if (command == "green") {
      color=(pixels.Color(0,150,0)); // Moderately bright green color.
    }
  }


  showOneCycle(color);
}
