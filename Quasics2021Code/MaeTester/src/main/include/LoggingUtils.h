#pragma once

#include <iostream>

#define LOG_EVERY_N_TIMES(n, msg)    \
  do {                               \
    static int counter = 0;          \
    if (counter == 0) {              \
      std::cerr << msg << std::endl; \
    }                                \
    counter = (counter + 1) % n;     \
  } while (false)
