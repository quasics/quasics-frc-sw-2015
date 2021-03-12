#pragma once

/// Generates logging output every N times that the code is executed.
/// (This is intended to help throttle debugging output to a level that is
/// reasonable/meaningful for developers/drivers.)
///
/// @param n    number of seconds between logging output
/// @param outputCmd  command to be run, producing logging output
#define LOG_EVERY_N_TIMES(n, outputCmd)       \
  {                                           \
    static int counter = -1;                  \
    if ((counter = (counter + 1) % n) == 0) { \
      outputCmd;                              \
    }                                         \
  }

/// Generates logging output every N seconds.
///
/// Intended for use within "Periodic" functions, etc., with an assumed
/// frequency of 50Hz (which is standard for WPILib).
///
/// @param n    number of seconds between logging output
/// @param outputCmd  command to be run, producing logging output
#define LOG_EVERY_N_SECONDS(n, outputCmd) LOG_EVERY_N_TIMES(((n)*50), outputCmd)