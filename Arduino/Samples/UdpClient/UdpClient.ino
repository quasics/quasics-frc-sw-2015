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
#include <EthernetUdp.h>

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

// Pin wired to an LED being controlled by remote commands.
// Note that we can't use LED_BUILTIN, because it's used by the Ethernet shield.
constexpr int LED_PIN = 7;


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
  // and turn on the light.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

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

////////////////////////////////////////////////////////////////////////////////////////
//
// Primary function
//
////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // Look for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    // No data, but pause briefly (to allow something to come in) before we bail out.
    delay(10);
    return;
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

  Serial.print("Received command: \"");
  Serial.print(packetBuffer);
  Serial.println("\"");

  if (strcmp(packetBuffer, "on") == 0) {
    Serial.println("Turn on the lights");
    digitalWrite(LED_PIN, HIGH);
  } else if (strcmp(packetBuffer, "off") == 0) {
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
