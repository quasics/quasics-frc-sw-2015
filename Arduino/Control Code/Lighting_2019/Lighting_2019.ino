#include <Adafruit_NeoPixel.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "PatternColor.h"

#define USE_NETWORK
#define ENABLE_LOGGING
#undef USE_RANDOM_MAC_ADDRESS

#ifdef ENABLE_LOGGING
  #define LOG_INLINE   Serial.print
  #define LOG          Serial.println
#else
  #define LOG_INLINE   (void)
  #define LOG          (void)
#endif

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;

const unsigned int kLocalPort = 10900;


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            7

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      25

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


//////////////////////////////////////////////////////////////////////////////////////////
// Color value definitions for functions from "LightingPatternsPlayground" code file

const uint32_t BLACK = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t RED = Adafruit_NeoPixel::Color(150, 0, 0);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0, 0, 150);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 150, 0);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(150, 150, 0);

////////////////////////////////////////////////////////////////////////////////////////
//
// Network and MAC address management
//
////////////////////////////////////////////////////////////////////////////////////////

#define MAC_LENGTH    6

// Enter a MAC address for your controller below (or generate a random one).
//
// Newer Ethernet shields have a MAC address printed on a sticker on the shield; if
// yours doesn't, you can pick a reasonably-random combination of 6 bytes, cross your
// fingers, and pray.
//
// See also: https://forum.arduino.cc/index.php?topic=243104.0
byte mac[MAC_LENGTH] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// A simple hash function from Robert Sedgwick's "Algorithms in C" book.
// Code taken from http://www.partow.net/programming/hashfunctions/
unsigned long int RSHash(const char* str)
{
   unsigned long int b    = 378551;
   unsigned long int a    = 63689;
   unsigned long int hash = 0;
   unsigned long int i    = 0;

   for (; *str != 0; ++str, ++i) {
      hash = hash * a + (*str);
      a    = a * b;
   }

   return hash;
}

// Function to generate a random MAC address.  (Reasonably safe of collisions, given
// the random seed being generated from the date/time at which the code was compiled.)
void generateMacAddress(byte mac[MAC_LENGTH]) {
  // Seed the random # generator, based on the compilation date/time.
  randomSeed(RSHash(__DATE__ " " __TIME__));

  for (int i = 0; i < MAC_LENGTH; ++i) {
    mac[i] = byte(random(256));
  }

#ifdef ENABLE_LOGGING
  LOG_INLINE("Generated MAC address: ");
  for (int i = 0; i < MAC_LENGTH; ++i) {
    LOG_INLINE(mac[i], HEX);
    LOG_INLINE(i < (MAC_LENGTH - 1) ? ':' : '\n');
  }
#endif
}

bool configureNetwork(byte mac[MAC_LENGTH], IPAddress* staticAddress = 0, IPAddress* staticDns = 0) {
  if (Ethernet.begin(mac) == 0) {
    LOG("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      LOG("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      return false;
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      LOG("Ethernet cable is not connected.");
      return false;
    }
    if (staticAddress && staticDns) {
      // try to congifure using IP address instead of DHCP:
      Ethernet.begin(mac, *staticAddress, *staticDns);
      return true;
    }
    return false;
  } else {
    LOG_INLINE("  DHCP assigned IP ");
    LOG(Ethernet.localIP());
    return true;
  }
}

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

void chase(PatternColor patternColor) {
  const uint32_t c = getColorValue(patternColor);
  const int delayval = 30; // delay for 3/100ths of a second

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, BLACK);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }
}

void pulse(PatternColor patternColor) {
  const int delayval = 5; // delay for 0.005 seconds

  // Ramp the lights up
  for (int b = 0; b <= 150; b = b + 10) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
      pixels.show();
      delay(delayval);
    }
  }

  // Ramp them back down
  for (int b = 150; b >= 0; b = b - 10) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
      pixels.show();
      delay(delayval);
    }
  }
}

void blinking(PatternColor patternColor) {
  const uint32_t c = getColorValue(patternColor);
  const int delayval = 400;   // Delay time in milliseconds (4/10ths sec)

  // Turn them all on
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);

  // Turn them all off
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, BLACK);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Lighting control code for 2019 robot.

void signalSetupErrorForever() {
  bool turnOn = true;
  while (true) {
    blinking(eYellow);
    turnOn = !turnOn;
  }
}

// Set up is called one time (at program start).
void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Running....");

  //////////////////////////////////
  // Set up networking
#ifdef USE_RANDOM_MAC_ADDRESS
  Serial.println("Generating random MAC address...");
  generateMacAddress(mac);
#endif

  // Start the Ethernet connection.
  Serial.println("Initializing Ethernet...");
  if (!configureNetwork(mac)) {
    Serial.println("Failed to configure network: I refuse to proceed....");
    signalSetupErrorForever();
  }

  // Start UDP.
  udp.begin(kLocalPort);
  Serial.print("Waiting for UDP packets on port ");
  Serial.println(kLocalPort);
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
#ifdef USE_NETWORK
  command = getCommandFromNetwork();
#else
  command = getCommandFromSerialMonitor();
#endif

  if (command != "") {
    Serial.println("Got: '" + command + "'");
    if (command == "red")  {
      patternColor = eRed;
    } else if (command == "blue") {
      patternColor = eBlue;
    } else if (command == "demo") {
      patternColor = eGreen;
    }
   else if (command == "Lifter" || command == "Both" || command == "Elevator"){
    patternName = command;
   }
  }

  // Apply whatever the current pattern is, using the current color.
  Serial.println("Applying pattern: " + patternName);
  if (patternName == "Lifter"){  
    pulse(patternColor);
  }
  else if (patternName == "Elevator"){ 
    blinking(patternColor);
  }  
  else {
    // By default, we'll show the "chase" pattern.
    chase(patternColor);
  }
}
