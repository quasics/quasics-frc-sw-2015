#ifndef CONFIGURATION_FLAGS_H
#define CONFIGURATION_FLAGS_H

/*
 * Compile-time configuration settings.  Muck with these CAREFULLY!!!
 */

// If defined, log lots of stuff to the Serial Monitor.
#define ENABLE_LOGGING

// If defined, use a randomly-generated MAC address.
#undef USE_RANDOM_MAC_ADDRESS

// If defined, allow a static IP address to be used if DHCP fails/is skipped.
// DO NOT USE IN COMPETITON!
#undef ALLOW_STATIC_IP_ADDRESS

// If defined, don't bother trying to configure network via DHCP.
// Note: only works if "ALLOW_STATIC_IP_ADDRESS" is also defined.
#undef SKIP_DHCP

#endif  // CONFIGURATION_FLAGS_H

