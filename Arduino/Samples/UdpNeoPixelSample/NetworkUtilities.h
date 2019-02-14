#ifndef NETWORK_UTILITIES_H
#define NETWORK_UTILITIES_H

#include <Ethernet.h>

// Defines the length of a MAC address (as an array of bytes).
const int  MAC_LENGTH = 6;

// Function to generate a random MAC address.  (Reasonably safe of collisions, given
// the random seed being generated from the date/time at which the code was compiled.)
void generateMacAddress(byte mac[MAC_LENGTH]);

// Configures network via DHCP if available, with an optional static IP/DNS configuration as fallback.
bool configureNetwork(byte mac[MAC_LENGTH], IPAddress* staticAddress = 0, IPAddress* staticDns = 0);

// Configures network using static IP/DNS configuration.
// Note: This is for testing purposes *only*, and should not be used in competition.
bool configureStaticNetwork(byte mac[MAC_LENGTH], IPAddress staticAddress, IPAddress staticDns);

bool getPacketData(EthernetUDP & udp, String & result);

void sendPacket(EthernetUDP & udp, IPAddress destination, unsigned short remotePort, const String & packetText);


#endif // NETWORK_UTILITIES_H

