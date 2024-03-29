#include "NetworkUtilities.h"

#include <Ethernet.h>
#include "Logging.h"

////////////////////////////////////////////////////////////////////////////////////////
//
// Network and MAC address management
//
////////////////////////////////////////////////////////////////////////////////////////

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

bool configureStaticNetwork(byte mac[MAC_LENGTH], IPAddress staticAddress, IPAddress staticDns) {
  LOG("Initializing Ethernet (static only)...");
  Ethernet.begin(mac, staticAddress, staticDns);
  
  // Check for Ethernet possible causes of failure
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    LOG("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    return false;
  }
  else if (Ethernet.linkStatus() == LinkOFF) {
    LOG("Ethernet cable is not connected.");
    return false;
  }

  Serial.print("Running at IP: ");
  Serial.println(Ethernet.localIP());
  return true;
}

bool configureNetwork(byte mac[MAC_LENGTH], IPAddress* staticAddress, IPAddress* staticDns) {
  LOG((staticAddress && staticDns)
        ? "Initializing Ethernet (DHCP/static fallback)..."
        : "Initializing Ethernet (DHCP only)...");

  if (!Ethernet.begin(mac)) {
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
    if (!(staticAddress && staticDns)) {
      LOG("- No static IP information provided: can't initialize network.");
      return false;
    }

    // Try to configure using static IP/DNS address
    LOG("- Falling back on static configuration");
    Ethernet.begin(mac, *staticAddress, *staticDns);
  }
  
  Serial.print("Running at IP: ");
  Serial.println(Ethernet.localIP());
  return true;
}

bool getPacketData(EthernetUDP & udp, String & result) {
#if !(defined( ALLOW_STATIC_IP_ADDRESS ) && defined( SKIP_DHCP ))
  // Make sure that we keep our lease on our IP address, if DHCP was enabled during startup.
  Ethernet.maintain();
#endif

  result = "";    // Clear the output

  // Look for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    // No data.
    return false;
  }

#ifdef ENABLE_LOGGING
  // Dump information on what we received.
  LOG_INLINE("\nReceived packet of size ");
  LOG_INLINE(packetSize);
  LOG_INLINE(" from ");
  IPAddress remote = udp.remoteIP();
  for (int i=0; i < 4; i++) {
    LOG_INLINE(remote[i], DEC);
    if (i < 3) {
      LOG_INLINE(".");
    }
  }
  LOG_INLINE(", port ");
  LOG(udp.remotePort());
#endif

  // Read in the packet's data
  result.reserve(packetSize);
  for (int i = 0; i < packetSize; ++i) {
    result += char(udp.read());
  }

  return true;
}

void sendPacket(EthernetUDP & udp, IPAddress destination, unsigned short remotePort, const String & packetText) {
  // send a reply to the IP address and port that sent us the packet we received
  udp.beginPacket(destination, remotePort);
  udp.write(packetText.c_str());
  udp.endPacket();
}

