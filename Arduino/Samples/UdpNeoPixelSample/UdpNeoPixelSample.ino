/*
   Sample program, demonstrating how UDP can be used for communications on an Arduino.

   This program will listen for UDP packets on the (aribitrary) port 10900.  Clients that
   want to send it data can either send it to that port *directly* on this address, or
   else can send it as a "broadcast" message using an address like "255.255.255.255" (and
   that port).

   The latter option is better for cases where you may not know what IP address the Arduino
   will have (e.g., when one is randomly assigned to a device).

   This program also provides a sample mechanism for generating a random MAC address, so as
   to reduce the risk of "collisions" with other devices on the network.  This functionality
   can be enabled by defining the "USE_RANDOM_MAC_ADDRESS" symbol; if this is not defined,
   a fixed MAC address will be used.
*/
#include <Ethernet.h>

#include "ConfigurationFlags.h"
#include "Logging.h"
#include "NetworkUtilities.h"
#include "NeoPixelControl.h"
#include "SerialUtilities.h"

////////////////////////////////////////////////////////////////////////////////////////
//
// File-scoped variables used for network setup/activity
//
////////////////////////////////////////////////////////////////////////////////////////

#ifndef SKIP_NETWORKING
// A wrapper for access to a UDP port, for use in receiving commands from remote machines.
static EthernetUDP udp;

// The UDP port on which we'll expect to receive networked commands.  All remote machines
// will need to know this port in order to broadcast their commands (and for the Arduino
// to "hear" the transmissions).
static const unsigned int kLocalPort = 10900;

// A default MAC address to be used for the controller.
//
// Newer Ethernet shields have a MAC address printed on a sticker on the shield; if
// yours doesn't, you can pick a reasonably-random combination of 6 bytes, cross your
// fingers, and pray.  (The generateMacAddress() can do the first one for you.)
//
// See also: https://forum.arduino.cc/index.php?topic=243104.0
static byte mac[MAC_LENGTH] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

#ifdef ALLOW_STATIC_IP_ADDRESS
// Predefined IP address to be used by the Arduino.
// Note that this is specific to when the Arduino is plugged into Mr. Healy's Macbook,
// and would likely need to be adjusted for use elsewhere.
static IPAddress staticIP(169, 254, 164, 177);

// Google's DNS server IP address.
static IPAddress staticDNS(8, 8, 8, 8);
#endif  // ALLOW_STATIC_IP_ADDRESS
#endif  // SKIP_NETWORKING


////////////////////////////////////////////////////////////////////////////////////////
//
// File-scoped variables used for NeoPixel control
//
////////////////////////////////////////////////////////////////////////////////////////

// Pin wired to control the NeoPixels.
// Note that pins 10-13 are used by the Ethernet Shield. (https://playground.arduino.cc/Main/ShieldPinUsage)
constexpr int NEOPIXEL_PIN = 7;

// How many LEDs are on the NeoPixel strip being controlled.
constexpr int NEOPIXEL_LENGTH = 24;

// The type of NeoPixel strip.
constexpr neoPixelType NEOPIXEL_TYPE = NEOPIXEL_RING_RGBW;

////////////////////////////////////////////////////////////////////////////////////////
//
// Set-up code, run at the start of the program.
//
////////////////////////////////////////////////////////////////////////////////////////

// Enable network access, and start listening for remote commands.
bool prepareNetwork(byte mac[MAC_LENGTH], EthernetUDP& udp, int localPort) {
#ifdef USE_RANDOM_MAC_ADDRESS
  Serial.println("Generating random MAC address...");
  generateMacAddress(mac);
#endif

  // Start the Ethernet connection.
#if defined( ALLOW_STATIC_IP_ADDRESS ) && defined( SKIP_DHCP )
  bool networkOK = configureStaticNetwork(mac, staticIP, staticDNS);
#elif defined( ALLOW_STATIC_IP_ADDRESS )
  bool networkOK = configureNetwork(mac, &staticIP, &staticDNS);
#else
  bool networkOK = configureNetwork(mac);
#endif
  if (!networkOK) {
    return false;
  }

  // Start listening for UDP packets on our designated port.
  udp.begin(localPort);
  Serial.print("Waiting for UDP packets on port ");
  Serial.println(localPort);
  return true;
}

// Standard Arduino setup function.
void setup() {
  // Open serial communications and wait for port to open.
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  initializeNeoPixels(NEOPIXEL_PIN, NEOPIXEL_LENGTH, NEOPIXEL_TYPE);

#ifndef SKIP_NETWORKING
  if (!prepareNetwork(mac, udp, kLocalPort)) {
    Serial.println("Failed to configure network: I refuse to proceed....");
    setNeoPixelMode(NeoPixelMode::eError);
    while (true) {
      stepNeoPixels();
      delay(1);
    }
  }
#endif  // SKIP_NETWORKING

  setNeoPixelMode(NeoPixelMode::eOn);
}

////////////////////////////////////////////////////////////////////////////////////////
//
// Primary function
//
////////////////////////////////////////////////////////////////////////////////////////

// Processes a single command.
void processOneCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "on") {
    Serial.println("- Turn on the lights");
    setNeoPixelMode(NeoPixelMode::eOn);
  } else if (cmd == "off") {
    Serial.println("- Turn off the lights");
    setNeoPixelMode(NeoPixelMode::eOff);
  } else {
    Serial.print("--- Unknown command: '");
    Serial.print(cmd);
    Serial.println("'");
  }
}

// Processes a set of 0 or more commands, following the format:
//
//      command;command2;command3;....
//
// with optional whitespace before/after each command, and semi-colons separating
// them.  (A final semicolon at the end is legal, but optional.)
void processCommandSet(const String & commandSet) {
  Serial.print("\nProcessing command set: \"");
  Serial.print(commandSet);
  Serial.println("\"");

  int lastStart = 0;
  for (int nextBreak = commandSet.indexOf(';'); nextBreak != -1; nextBreak = commandSet.indexOf(';', (lastStart = nextBreak + 1))) {
    String cmd = commandSet.substring(lastStart, nextBreak);
    processOneCommand(cmd);
  }

  String cmd = commandSet.substring(lastStart);
  processOneCommand(cmd);
}

// Standard Arduino main function.
void loop() {
  // Update the NeoPixel strip (if needed).
  stepNeoPixels();

  // Get and process any commands.
  String commandSet;
  if (getSerialCommand(Serial, commandSet)) {
    processCommandSet(commandSet);
  }

#ifndef SKIP_NETWORKING
  if (getPacketData(udp, commandSet)) {
    processCommandSet(commandSet);
    // Optional: send a reply to the IP address and port that sent us the packet we received
    //     sendPacket(udp, udp.remoteIP(), udp.remotePort(), "ACK: " + commandSet);
  }
#endif  // SKIP_NETWORKING
}

