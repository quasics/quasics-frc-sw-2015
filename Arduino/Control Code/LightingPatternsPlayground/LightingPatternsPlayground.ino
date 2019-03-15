#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 30; // delay for half a second

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.

   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

  }
}

void loop() {
  const uint32_t RED = pixels.Color(150,0,0);
  const uint32_t BLUE = pixels.Color(0,0,150);
  const uint32_t GREEN = pixels.Color(0,150,0);
  chase(RED);
  chase(BLUE);
  chase(GREEN);
  pulse("RED");
  pulse("BLUE");
  pulse("GREEN");
  blinking(RED);
  blinking(BLUE);
  blinking(GREEN);
}

void chase(uint32_t c) {

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i,c); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }
}

void pulse(String colorName){
    for(int b = 0; b<150; b = b + 10){
       for(int i=0;i<NUMPIXELS;i++){
      if(colorName == "GREEN"){
        pixels.setPixelColor(i, pixels.Color(0, b, 0));
      }
      if(colorName == "RED"){
        pixels.setPixelColor(i, pixels.Color(b, 0, 0));
      }
      if(colorName == "BLUE"){
        pixels.setPixelColor(i, pixels.Color(0, 0, b));
      }
      delay(5);
      pixels.show();
      }
    }
    for(int b = 150; b>0; b = b - 10){
       for(int i=0;i<NUMPIXELS;i++){
      if(colorName == "GREEN"){
        pixels.setPixelColor(i, pixels.Color(0, b, 0));
      }
      if(colorName == "RED"){
        pixels.setPixelColor(i, pixels.Color(b, 0, 0));
      }
      if(colorName == "BLUE"){
        pixels.setPixelColor(i, pixels.Color(0, 0, b));
      }
      delay(5);
      pixels.show();
      }
    }
}

void blinking(uint32_t c) {

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i,c); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

  }
  delay(400); // Delay for a period of time (in milliseconds).

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

  }
   delay(400); // Delay for a period of time (in milliseconds).
}
