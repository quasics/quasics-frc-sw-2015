#pragma once

#define ENABLE_DRIVE_BASE_LEAD_FOLLOW
#define BACK_MOTORS_ARE_LEADERS

// See bug filed at https://github.com/PhotonVision/photonvision/issues/1222.
#define LEAK_VISION_TO_WORK_AROUND_CLEANUP_BUG

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
// Targeting MacOS (for emulation/unit testing)
#define ENABLE_XRP
#elif defined(_WIN32)
// Targeting Windows (for emulation/unit testing)
#define ENABLE_XRP
#else
// (*shrug*) Assume Windows (for emulation/unit testing), but we could also use
// the lists at
// https://github.com/cpredef/predef/blob/master/OperatingSystems.md
#define ENABLE_XRP
#endif
