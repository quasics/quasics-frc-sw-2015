// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT  120  // 60 on each side

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed, with options including:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (NeoPixel GRBW products)


// setup() function -- runs once at startup --------------------------------

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}


// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  runningStrip(strip.Color(0, 255, 0), 5, 50);
}

// Sets all pixels to BG color (but does not apply the changes).
void clearStrip(uint32_t bgColor) {
  for(uint32_t i = 0; i < strip.numPixels(); ++i) {
    strip.setPixelColor(i, bgColor);
  }
}

// Renders a lit sequence of <length> pixels, starting at position <start>,
// but does not apply the changes.
// (Note that this assumes all pixels not affected are set to BG color.)
void lightOneSegment(uint32_t color, uint32_t start, uint32_t length) {
  for(uint32_t i = 0; i < length; ++i) {
    strip.setPixelColor((start+i) % strip.numPixels(), color);
  }
}

void runningStrip(uint32_t color, uint32_t length, int wait) {
  const uint32_t offset = strip.numPixels() / 3;

  for (uint32_t start = 0; start < strip.numPixels(); ++start) {
    clearStrip(strip.Color(0, 0, 0));
    lightOneSegment(color, start, length);
    lightOneSegment(color, (start + offset) % strip.numPixels(), length);
    lightOneSegment(color, (start + offset * 2) % strip.numPixels(), length);
    strip.show();
    delay(wait);
  }
}
