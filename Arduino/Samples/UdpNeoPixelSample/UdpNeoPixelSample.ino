/*
 * Sample program, demonstrating how UDP can be used for communications on an Arduino.
 * 
 * This program will listen for UDP packets on the (aribitrary) port 10900.  Clients that
 * want to send it data can either send it to that port *directly* on this address, or
 * else can send it as a "broadcast" message using an address like "255.255.255.255" (and
 * that port).
 * 
 * The latter option is better for cases where you may not know what IP address the Arduino
 * will have (e.g., when one is randomly assigned to a device).
 * 
 * This program also provides a sample mechanism for generating a random MAC address, so as
 * to reduce the risk of "collisions" with other devices on the network.  This functionality
 * can be enabled by defining the "USE_RANDOM_MAC_ADDRESS" symbol; if this is not defined,
 * a fixed MAC address will be used.
 */
#include <Ethernet.h>

#include "ConfigurationFlags.h"
#include "Logging.h"
#include "NetworkUtilities.h"
#include "NeoPixelControl.h"

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;

const unsigned int kLocalPort = 10900;

// Pin wired to control the NeoPixels.
// Note that pins 10-13 are used by the Ethernet Shield. (https://playground.arduino.cc/Main/ShieldPinUsage)
constexpr int NEOPIXEL_PIN = 7;

// How many LEDs are on the NeoPixel strip being controlled.
constexpr int NEOPIXEL_LENGTH = 24;

// The type of NeoPixel strip.
constexpr neoPixelType NEOPIXEL_TYPE = NEOPIXEL_RING_RGBW;

// Enter a MAC address for your controller below (or generate a random one).
//
// Newer Ethernet shields have a MAC address printed on a sticker on the shield; if
// yours doesn't, you can pick a reasonably-random combination of 6 bytes, cross your
// fingers, and pray.
//
// See also: https://forum.arduino.cc/index.php?topic=243104.0
byte mac[MAC_LENGTH] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

#ifdef ALLOW_STATIC_IP_ADDRESS
IPAddress staticIP(169, 254, 164, 177);
IPAddress staticDNS(8, 8, 8, 8);    // Google's DNS server
#endif

////////////////////////////////////////////////////////////////////////////////////////
//
// Set-up code, run at the start of the program.
//
////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Open serial communications and wait for port to open.
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  initializeNeoPixels(NEOPIXEL_PIN, NEOPIXEL_LENGTH, NEOPIXEL_TYPE);

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
    Serial.println("Failed to configure network: I refuse to proceed....");
    setNeoPixelMode(NeoPixelMode::eError);
    while(true) {
      stepNeoPixels();
      delay(1);
    }
  }

  // Start listening for UDP packets on our designated port.
  udp.begin(kLocalPort);
  Serial.print("Waiting for UDP packets on port ");
  Serial.println(kLocalPort);

  setNeoPixelMode(NeoPixelMode::eOn);
}

////////////////////////////////////////////////////////////////////////////////////////
//
// Primary function
//
////////////////////////////////////////////////////////////////////////////////////////

void loop() {
#if defined( ALLOW_STATIC_IP_ADDRESS ) && defined( SKIP_DHCP )
  // Make sure that we keep our lease on our IP address, if DHCP was enabled during startup.
  Ethernet.maintain();
#endif

  stepNeoPixels();

  String command;
  if (!getPacketData(udp, command)) {
    // No data, but pause briefly (to allow something to come in) before we bail out.
    delay(10);
    return;
  }

  Serial.print("Received command: \"");
  Serial.print(command);
  Serial.println("\"");

  if (command == "on") {
    Serial.println("Turn on the lights");
    setNeoPixelMode(NeoPixelMode::eOn);
  } else if (command == "off") {
    Serial.println("Turn off the lights");
    setNeoPixelMode(NeoPixelMode::eOff);
  } else {
    Serial.println("  --- Unknown command");
  }

//  // send a reply to the IP address and port that sent us the packet we received
//  udp.beginPacket(udp.remoteIP(), udp.remotePort());
//  udp.write("ACK");
//  udp.endPacket();
}

