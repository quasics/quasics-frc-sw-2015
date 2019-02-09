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

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;

const unsigned int kLocalPort = 10900;

// Pin wired to an LED being controlled by remote commands.
// Note that we can't use LED_BUILTIN, because it's used by the Ethernet shield.
constexpr int LED_PIN = 7;

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

void signalSetupErrorForever() {
  bool turnOn = true;
  while (true) {
    digitalWrite(LED_PIN, turnOn ? HIGH : LOW);
    turnOn = !turnOn;
    delay(100);                       // wait for 1/10th of a second
  }
}

void setup() {
  // Open serial communications and wait for port to open.
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

#ifdef USE_RANDOM_MAC_ADDRESS
  Serial.println("Generating random MAC address...");
  generateMacAddress(mac);
#endif

  // Initialize the designated digital pin as an output (controlling an external LED),
  // and turn off the light.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Start the Ethernet connection.
  Serial.println("Initializing Ethernet...");
#if defined( ALLOW_STATIC_IP_ADDRESS ) && defined( SKIP_DHCP )
  bool networkOK = configureStaticNetwork(mac, &staticIP, &staticDNS);
#elif defined( ALLOW_STATIC_IP_ADDRESS )
  bool networkOK = configureNetwork(mac, staticIP, staticDNS);
#else
  bool networkOK = configureNetwork(mac);
#endif
  if (!networkOK) {
    Serial.println("Failed to configure network: I refuse to proceed....");
    signalSetupErrorForever();
  }

  // Start listening for UDP packets on our designated port.
  udp.begin(kLocalPort);
  Serial.print("Waiting for UDP packets on port ");
  Serial.println(kLocalPort);

  // Turn on the light to show that we're ready to proceed
  digitalWrite(LED_PIN, HIGH);
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
    digitalWrite(LED_PIN, HIGH);
  } else if (command == "off") {
    Serial.println("Turn off the lights");
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("  --- Unknown command");
  }

//  // send a reply to the IP address and port that sent us the packet we received
//  udp.beginPacket(udp.remoteIP(), udp.remotePort());
//  udp.write("ACK");
//  udp.endPacket();
}
