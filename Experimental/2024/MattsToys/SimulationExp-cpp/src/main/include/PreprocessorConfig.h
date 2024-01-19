#pragma once

// DEFINE this symbol to enable using the CTRE Pigeon2 (or other CTRE devices).
// UNDEFINE it to turn off CTRE use (e.g., if they haven't released the
// vendordeps yet).
#define ENABLE_CTRE

// The XRP classes aren't available when compiling for the RoboRio (only when
// building for simulation on a host platform).
#if defined(__linux__)
// Targeting the RoboRio
#undef ENABLE_XRP
#elif defined(__APPLE__)
// Targeting MacOS
#define ENABLE_XRP
#else
// (*shrug*) Assume Windows....
#define ENABLE_XRP
#endif
