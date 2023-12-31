#ifndef CONFIGURATION_FLAGS_H
#define CONFIGURATION_FLAGS_H

/*
 * Compile-time configuration settings.  Muck with these CAREFULLY!!!
 */

// If defined, log lots of stuff to the Serial Monitor.
#undef ENABLE_LOGGING

// If defined, use a randomly-generated MAC address.
//
// *** BE CAREFUL WHEN ENABLING THIS ***
//
// When this is enabled and a program is redeployed enough times, it is possible
// for a DHCP server to run out of available addresses to assign (until it's
// restarted).
#undef USE_RANDOM_MAC_ADDRESS

// If defined, allow a static IP address to be used if DHCP fails/is skipped.
//
// *** DON'T ENABLE THIS UNLESS YOU REALLY KNOW WHAT YOU'RE DOING ***
#undef ALLOW_STATIC_IP_ADDRESS

// If defined, don't bother trying to configure network via DHCP.
// Note: only works if "ALLOW_STATIC_IP_ADDRESS" is also defined.
#undef SKIP_DHCP

// If defined, don't do *anything* with networking.  (Useful when
// testing on a device without an Ethernet shield.)
#undef SKIP_NETWORKING

// The UDP port on which we'll expect to receive networked commands.  All remote machines
// will need to know this port in order to broadcast their commands (and for the Arduino
// to "hear" the transmissions).
#define kLocalPort 10900

// Predefined IP address to be used by the Arduino when static IP use is allowed.
//
// Note that this is specific to when the Arduino is plugged into Mr. Healy's Macbook,
// and would likely need to be adjusted for use elsewhere.  (For example, if it was
// being used on a robot at competition, we'd probably want to use something like
// "10, 26, 56, 19", though we'd need to make sure that it didn't conflict with
// anything else on the robot.)
#define STATIC_IP 169, 254, 164, 177

// Google's DNS server IP address.
#define STATIC_DNS_IP 8, 8, 8, 8


#endif  // CONFIGURATION_FLAGS_H
