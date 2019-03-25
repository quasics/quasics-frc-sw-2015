#include <Adafruit_NeoPixel.h>
#include "PatternColor.h"
#include "NetworkUtilities.h"
#include "Logging.h"

//////////////////////////////////////////////////////////////////////////////////////////
// Global variables used for networking

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;

// Which port is being used to send commands to the Arduino via the network?
const unsigned int kLocalPort = 10900;

// Enter a MAC address for your controller below (or generate a random one).
//
// Newer Ethernet shields have a MAC address printed on a sticker on the shield; if
// yours doesn't, you can pick a reasonably-random combination of 6 bytes, cross your
// fingers, and pray.
//
// See also: https://forum.arduino.cc/index.php?topic=243104.0
byte mac[MAC_LENGTH] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Number of times that we'll retry network initialization.
const unsigned int kNumNetworkInitRetries = 100;


//////////////////////////////////////////////////////////////////////////////////////////
// Global variables used for NeoPixel control

// Which pin on the Arduino is connected to the NeoPixels?
const int PIN = 6;

// How many NeoPixels are attached to the Arduino?
const int NUMPIXELS = 25;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

const PatternColor ERROR_COLOR = eGreen;


//////////////////////////////////////////////////////////////////////////////////////////
// Color value definitions for functions from "LightingPatternsPlayground" code file

const uint32_t BLACK = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t RED = Adafruit_NeoPixel::Color(150, 0, 0);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0, 0, 150);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 150, 0);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(150, 150, 0);


//////////////////////////////////////////////////////////////////////////////////////////
// Utility functions from "LightingPatternsPlayground" code file
//
// These will allow us to turn a given PatternColor into a color value that can be used
// to control a NeoPixel strip.

uint32_t getColorValue(PatternColor patternColor) {
  if (patternColor == eRed)
    return RED;
  if (patternColor == eGreen)
    return GREEN;
  if (patternColor == eBlue)
    return BLUE;
  if (patternColor == eYellow)
    return YELLOW;
  if (patternColor == eBlack)
    return BLACK;

  // This shouldn't ever happen, but if it does, we'll be able to get some warning.
  Serial.print("Unrecognized PatternColor: ");
  Serial.print(int(patternColor));
  Serial.println(" --- Falling back on YELLOW.");
  return YELLOW;
}

uint32_t getColorValueUsingIntensity(PatternColor patternColor, int intensity) {
  if (patternColor == eGreen) {
    return pixels.Color(0, intensity, 0);
  }
  else if (patternColor == eRed) {
    return pixels.Color(intensity, 0, 0);
  }
  else if (patternColor == eBlue) {
    return pixels.Color(0, 0, intensity);
  }
  else {
    // Just assume it's yellow
    return pixels.Color(intensity, intensity, 0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// Lighting control functions from "LightingPatternsPlayground" code file
//
// These will allow us to display a given pattern (1x), using a given PatternColor.

void showSolid(PatternColor patternColor) {
  const uint32_t c = getColorValue(patternColor);
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void chaseLights(PatternColor primaryColor, PatternColor secondaryColor = eBlack) {
  const uint32_t c = getColorValue(primaryColor);
  const uint32_t clearColor = getColorValue(secondaryColor);
  const int delayval = 25; // delay between steps in the chase

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, clearColor);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }
}

void pulseLights(PatternColor patternColor) {
  const int delayval = 18; // delay betweek steps "up" and "down"
  const int stepSize = 5;

  // Ramp the lights up
  for (int b = 0; b <= 150; b += stepSize) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
    }
    pixels.show();
    delay(delayval);
  }

  // Ramp them back down
  for (int b = 150; b >= 0; b -= stepSize) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
    }
    pixels.show();
    delay(delayval);
  }
}

void blinkLights(PatternColor patternColor, PatternColor secondaryColor = eBlack) {
  const uint32_t c = getColorValue(patternColor);
  const uint32_t clearColor = getColorValue(secondaryColor);
  const int delayval = 400;   // Delay time in milliseconds

  // Turn them all on
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);

  // Turn them all off
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, clearColor);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Lighting control code for 2019 robot.

void signalSetupErrorForever() {
  bool turnOn = true;
  while (true) {
    blinkLights(ERROR_COLOR);
    turnOn = !turnOn;
  }
}

bool setupNetworking() {
#ifdef USE_RANDOM_MAC_ADDRESS
  Serial.println("Generating random MAC address...");
  generateMacAddress(mac);
#endif

  bool configuredOK = false;
  const unsigned long startTime = millis();
  for(int tryNumber = 0; !configuredOK && tryNumber <= kNumNetworkInitRetries; ++tryNumber) {
    // Start the Ethernet connection.
    Serial.print("\nInitializing Ethernet (pass #");
    Serial.print(tryNumber);
    Serial.println(")...");

    configuredOK = configureNetwork(mac);
    const unsigned long timeNow = millis();
    LOG_INLINE("Elapsed time: ");
    LOG_INLINE((timeNow - startTime) / 1000);
    LOG_INLINE(" sec.");
    Serial.println(configuredOK  ? "Done!" : "Failed.");
    if (!configuredOK) {
      blinkLights(eYellow, eGreen);
    }
  }

  if (!configuredOK) {
    return false;
  }

  // Start UDP.
  udp.begin(kLocalPort);
  Serial.print("Waiting for UDP packets on port ");
  Serial.println(kLocalPort);
  return true;
}

// Set up is called one time (at program start).
void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  showSolid(eGreen);
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Running....");

#ifdef SKIP_NETWORKING
  Serial.println("Note: Networking is disabled");
#else
  if (!setupNetworking()) {
    Serial.println("Failed to configure network: I refuse to proceed....");
    signalSetupErrorForever();
  }
#endif
}

// Returns a command (i.e., a line of text entered by the user) obtained from the
// Serial Monitor, or the empty string if we don't have a command to process.
String getCommandFromSerialMonitor() {
  // Buffer used to hold input from the user that isn't fully usable as a command
  // yet (e.g., we haven't seen the "\n" or "\r" at the end of input from the
  // Serial Monitor window.
  //
  // This is "static", so it will retain its data between one call to this function
  // and the next.
  static String text;

  // Process any data from the serial monitor, and see if we have a full command yet.
  while (Serial.available() > 0) {
    char nextChar = Serial.read();
    if (nextChar != '\n' && nextChar != '\r') {
      text += nextChar;
    } else {
      String command = text;
      text = "";
      return command;
    }
  }

  // We don't seem to have a full command, so return an empty string.
  return "";
}

String getCommandFromNetwork() {
  // Make sure that we keep our lease on our IP address, if we were set up using DHCP.
  Ethernet.maintain();

  // Look for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    return "";
  }

  // Read in the packet's data
  static char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
  int len = udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
  if (packetBuffer[len - 1] != 0) {
    // Null-terminate the string, so that it's safe to use "normally".
    // (But be paranoid about it.)
    if (len < UDP_TX_PACKET_MAX_SIZE) {
      packetBuffer[len] = 0;
    } else {
      packetBuffer[len - 1] = 0;
    }
  }
  return packetBuffer;
}

// Loop function is called over and over and over (after setup() completes).
void loop() {
  // Holds the current color to be used by the LED strip.
  //
  // This is "static", so it will retain its data between one call to this function
  // and the next.  The assignment below will *only* happen the first time that the
  // function is executed; after that, it will keep the value it had the *last* time
  // that the function ran.  (So the pattern color will stick around between calls.)
  static PatternColor patternColor = eGreen;
  static String patternName = "Both";

  // See if we have a command to be executed; if we do, then process it (e.g., to change
  // the current pattern color).
  String command;
#ifdef SKIP_NETWORKING
  command = getCommandFromSerialMonitor();
#else
  command = getCommandFromNetwork();
#endif

  if (command != "") {
    Serial.println("Got: '" + command + "'");
    if (command.equalsIgnoreCase("red"))  {
      patternColor = eRed;
    } else if (command.equalsIgnoreCase("blue")) {
      patternColor = eBlue;
    } else if (command.equalsIgnoreCase("demo")) {
      patternColor = eGreen;
    }
   else if (command.equalsIgnoreCase("Lifter") || command.equalsIgnoreCase("Both") || command.equalsIgnoreCase("Elevator")){
    patternName = command;
   }

   // Note: during match play, the commands can come in from the controllers
   // in rapid succession (e.g., if we're quickly changing modes on the
   // elevator), so we'll skip running a cycle on the lights just yet, in
   // case there's another command pending.
   return;
  }

  // Apply whatever the current pattern is, using the current color.
  Serial.println("Applying pattern: " + patternName);
  if (patternName.equalsIgnoreCase("Lifter")){  
    pulseLights(patternColor);
  }
  else if (patternName.equalsIgnoreCase("Elevator")){ 
    blinkLights(patternColor);
  }  
  else {
    // By default, we'll show the "chase" pattern.
    chaseLights(patternColor);
  }
}
