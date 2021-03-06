#pragma once

#define LOG_EVERY_N_TIMES(n, outputCmd)       \
  {                                           \
    static int counter = -1;                  \
    if ((counter = (counter + 1) % n) == 0) { \
      outputCmd;                              \
    }                                         \
  }
