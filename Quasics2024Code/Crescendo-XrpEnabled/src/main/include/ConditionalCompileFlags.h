// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// DEFINE this symbol to enable using the CTRE Pigeon2 (or other CTRE devices).
// UNDEFINE it to turn off CTRE use (e.g., if they haven't released the
// vendordeps yet).
#define ENABLE_CTRE

#undef USING_MAE
#undef USING_SALLY
#define USING_MARGARET

#define ENABLE_FULL_ROBOT_FUNCTIONALITY

#undef ENABLE_INTAKE_TESTING

#undef ENABLE_VISION_TESTING

#undef LEAK_VISION_WORKAROUND

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
