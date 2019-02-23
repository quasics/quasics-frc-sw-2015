/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/LightingSubsystem.h"
#include "commands/AutoLighting.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

constexpr unsigned short USE_PORT = 10900;

//////////////////////////////////////////////////////////////////////////////
// "Boilerplate" code, used to set up a socket that can send command packets,
// and to build the destination address we'll send them to.

namespace {
// Returns a reference to the socket, or -1 on failure.
int setupUdpSocket() {
  // Try to get a socket
  int sock;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    return -1;
  }
  int broadcast = 1; /* Note: on some platforms, this may need to be "char" */
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast,
                 sizeof broadcast) == -1) {
    ::close(sock);
    return -1;
  }
  return sock;
}

// Translates the string version of an IP address (plus a port number) into
// the underlying data format that's required when we send messages.
//
// This is used to build up the destination address that we'll be sending
// our commands to (which in our case is expected to be a "broadcast" address
// that any number of computers could be listening to, but in practice will
// just be the Arduino controlling our lights).
bool getDestinationAddress(sockaddr_in &result, std::string ipAddress,
                           unsigned short port) {
  memset((char *)&result, 0, sizeof(result));
  if (inet_aton(ipAddress.c_str(), &result.sin_addr) == 0) {
    return false;
  }
  result.sin_family = AF_INET;
  result.sin_port = htons(port);
  return true;
}
}  // namespace

//////////////////////////////////////////////////////////////////////////////
// Basic definition of standard "Subsystem" stuff for the Lighting control.

// Constructor, used to build a LightingSubsystem object.
LightingSubsystem::LightingSubsystem()
    : Subsystem("LightingSubsystem"), udpSocket(setupUdpSocket()) {
  if (udpSocket == -1) {
    // We weren't able to create/set up the socket.
    std::cerr
        << "***** Warning: Failed to allocate socket for Lighting subsystem!\n";
  } else if (!getDestinationAddress(broadcastAddress, "255.255.255.255",
                                    USE_PORT)) {
    std::cerr << "***** Warning: Failed to get broadcast address for Lighting "
                 "subsystem!\n";

    // If we don't know who to broadcast to, we might as well close up the
    // socket, since we won't be able to use it later.
    std::cerr << "               (Closing socket....)\n";
    ::close(udpSocket);
    udpSocket = -1;
  }
}

// Destructor, used to clean up when the Subsystem goes away.
LightingSubsystem::~LightingSubsystem() {
  // Be polite, and close the socket now that we're done with it.
  if (udpSocket != -1) {
    ::close(udpSocket);
    udpSocket = -1;
  }
}

// Set the default command for the subsystem.
void LightingSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new AutoLighting);
}

//////////////////////////////////////////////////////////////////////////////
// Functions to be used by Commands to do things with the lights will go here.

// Sends the specified command out (via broadcast) to the Arduino.
bool LightingSubsystem::sendCommandToArduino(const std::string &cmd) {
  if (udpSocket == -1) {
    std::cerr << "Warning: Can't send '" << cmd << "' to Arduino (no socket)\n";
    return false;
  }

  const int bytesToSend = cmd.length() + 1;
  return (sendto(udpSocket, cmd.c_str(), bytesToSend, 0,
                 reinterpret_cast<const sockaddr *>(&broadcastAddress),
                 sizeof(broadcastAddress)) == bytesToSend);
}
