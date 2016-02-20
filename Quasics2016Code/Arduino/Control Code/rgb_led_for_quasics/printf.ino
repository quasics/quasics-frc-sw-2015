#include <stdarg.h>
void p(const char *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end (args);
  Serial.print(buf);
}

void fp(HardwareSerial& serial, const char *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end (args);
  serial.print(buf);
}

/** Sample use: p(F("Best to store long strings to flash to save %s"),"memory"); */
void p(const __FlashStringHelper *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
  va_end(args);
  Serial.print(buf);
}

void fp(HardwareSerial& serial, const __FlashStringHelper *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
  va_end(args);
  serial.print(buf);
}

