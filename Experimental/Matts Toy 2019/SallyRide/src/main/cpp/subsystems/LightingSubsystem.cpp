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

namespace {
// Returns a reference to the socket, or -1 on failure.
int setupUdpSocket() {
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

bool getBroadcastAddress(sockaddr_in &result, std::string ipAddress,
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

LightingSubsystem::LightingSubsystem()
    : Subsystem("ExampleSubsystem"), udpSocket(setupUdpSocket()) {
  if (udpSocket == -1) {
    std::cerr
        << "***** Warning: Failed to allocate socket for Lighting subsystem!\n";
  }
  if (!getBroadcastAddress(broadcastAddress, "255.255.255.255", USE_PORT)) {
    std::cerr << "***** Warning: Failed to get broadcast address for Lighting "
                 "subsystem!\n";
  }
}

LightingSubsystem::~LightingSubsystem() {
  if (udpSocket != -1) {
    ::close(udpSocket);
    udpSocket = -1;
  }
}

void LightingSubsystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new AutoLighting);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
bool LightingSubsystem::sendCommandToArduino(std::string cmd) {
  if (udpSocket == -1) {
    std::cerr << "Warning: Can't send '" << cmd << "' to Arduino (no socket)\n";
    return false;
  }

  return (sendto(udpSocket, cmd.c_str(), cmd.length() + 1, 0,
                 reinterpret_cast<const sockaddr *>(&broadcastAddress),
                 sizeof(broadcastAddress)) == -1);
}
