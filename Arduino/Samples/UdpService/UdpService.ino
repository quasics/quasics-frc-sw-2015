/*
   Sample program, demonstrating how UDP can be used for communications on an Arduino.

   This program will listen for UDP packets on the (aribitrary) port 10900.  Clients that
   want to send it data can either send it to that port *directly* on this address, or
   else can send it as a "broadcast" message using an address like "255.255.255.255" (and
   that port).

   The latter option is better for cases where you may not know what IP address the Arduino
   will have (e.g., when one is assigned to a device by a DHCP server, such as the one
   running on the robot's radio).

   This program also provides a sample mechanism for generating a random MAC address, so as
   to reduce the risk of "collisions" with other devices on the network.  This functionality
   can be enabled by defining the "USE_RANDOM_MAC_ADDRESS" symbol; if this is not defined,
   a fixed MAC address will be used.

   For the purposes of testing core logic, the program can also handle commands received via
   the Serial monitor.
*/
#include <Ethernet.h>

#include "ConfigurationFlags.h"
#include "Logging.h"
#include "NetworkUtilities.h"
#include "SerialUtilities.h"


////////////////////////////////////////////////////////////////////////////////////////
//
// File-scoped variables used for network setup/activity
//
////////////////////////////////////////////////////////////////////////////////////////

#ifndef SKIP_NETWORKING
// A wrapper for access to a UDP port, for use in receiving commands from remote
// machines.
static EthernetUDP udp;

// A default MAC address to be used for the controller, if we don't generate one
// randomly.
//
// Newer Ethernet shields have a MAC address printed on a sticker on the shield; if
// yours doesn't, you can pick a reasonably-random combination of 6 bytes, cross your
// fingers, and pray.  (The generateMacAddress() can do the first one for you.)
//
// See also: https://forum.arduino.cc/index.php?topic=243104.0
static byte mac[MAC_LENGTH] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
#endif  // SKIP_NETWORKING


////////////////////////////////////////////////////////////////////////////////////////
//
// Set-up code, run at the start of the program.
//
////////////////////////////////////////////////////////////////////////////////////////

// Enable network access, and start listening for remote commands.
bool prepareNetwork(byte mac[MAC_LENGTH], EthernetUDP& udp, int localPort) {
  DEFINE_STATIC_IP_CONFIG();

  // Start the Ethernet connection.
  const bool networkOK = 
#if defined( ALLOW_STATIC_IP_ADDRESS ) && defined( SKIP_DHCP )
      configureStaticNetwork(mac, staticIP, staticDNS)
#elif defined( ALLOW_STATIC_IP_ADDRESS )
      configureNetwork(mac, &staticIP, &staticDNS)
#else
      configureNetwork(mac)
#endif
    ;
  if (!networkOK) {
    return false;
  }

  // Start listening for UDP packets on our designated port.
  udp.begin(localPort);
  Serial.print("Listening on port ");
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

#ifndef SKIP_NETWORKING
#ifdef USE_RANDOM_MAC_ADDRESS
  generateMacAddress(mac);
#endif
  // Try connecting to the network some number of times before declaring
  // outright failure.  This is intended to cover cases like the Ethernet
  // cable not being plugged in right away when the Arduino is powered up,
  // or the DHCP server not being ready right away (e.g., giving the radio
  // on a robot a chance to power up, too), etc.
  bool networkUnavailable = true;
  for(int i = 0; networkUnavailable && i < kMaxTriesForNetworkConfig; ++i) {
    if (prepareNetwork(mac, udp, kLocalPort)) {
      networkUnavailable = false;
    } else {
      // Wait before trying again....
      LOG("Network init failed...");
      delay(100);
    }
  }
  if (networkUnavailable) {
    Serial.println("Halting! (Network setup failed)");
    while (true) {
      delay(1);
    }
  }
#endif  // SKIP_NETWORKING
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
    Serial.println("- Turn on");
  } else if (cmd == "off") {
    Serial.println("- Turn off");
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
  Serial.print("\nProcessing\: \"");
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