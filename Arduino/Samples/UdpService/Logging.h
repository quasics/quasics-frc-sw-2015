#ifndef LOGGING_H
#define LOGGING_H

#include "ConfigurationFlags.h"

#ifdef ENABLE_LOGGING
  #define LOG_INLINE   Serial.print
  #define LOG          Serial.println
#else
  #define LOG_INLINE   (void)
  #define LOG          (void)
#endif

#endif  // LOGGING_H
