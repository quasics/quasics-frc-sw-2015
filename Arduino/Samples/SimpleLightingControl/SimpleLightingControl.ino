/*
 * A simple lighting control demo, which includes basic buffering of the I/O and
 * turning the on-board LED on/off, based on receiving the commands "On" or "Off".
 *
 * This code will build for boards that have only one serial port (e.g., Uno and
 * Nano), or that have multiple ports (e.g., Mega).
 * 
 * Notes:
 *   * Commands can be received either from Serial (which is the default serial
 *     port, either USB or the "TX/RX" pins if USB isn't connected) or from
 *     Serial1 (the "TX1/RX1" pins) on boards that have that.
 *   * Commands must be terminated with either a "\n" (newline) or ";", or else
 *     they won't be parsed correctly.  If they're coming in from the Serial
 *     Monitor (to Serial), then you can either type a ";" at the end, or else
 *     set the Monitor up to automatically send newlines.  If they're coming in
 *     from a remote device (over Serial1), then the sender will need to include
 *     a terminating character after the command.
 */

////////////////////////////////////////////////////////////////////////////////
// Compile-time flags to enable/disable some blocks of code.

// Tries to detect if we're building for an Arduino with support for a
// second serial port (e.g., a Mega), vs. hardware without that support
// (e.g., an Uno/Nano).
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #undef HAVE_HWSERIAL1
#elif defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
  #define HAVE_HWSERIAL1
#else
  // Unrecognized board type: There are others with Serial1, but we'll assume not
  // for now.
  #undef HAVE_HWSERIAL1
#endif

#define ENABLE_DEBUGGING_OUTPUT


////////////////////////////////////////////////////////////////////////////////
// The actual program starts here.

void setup() {
  // Initialize serial port(s).
  Serial.begin(115200);
#ifdef HAVE_HWSERIAL1
  Serial1.begin(115200);
#endif

#ifdef ENABLE_DEBUGGING_OUTPUT
  // Display start-up message.
  Serial.println("SerialConsole now running....");
#endif  // ENABLE_DEBUGGING_OUTPUT

  // Enable using the built-in LED.
  pinMode(LED_BUILTIN, OUTPUT);
}

// Function to decide if a character marks the termination of a line/command.
bool isEndOfLineMarker(char c) {
  return c == '\n' || c == ';';
}

// Execute the specified command.
void processCommand(String cmd) {
  cmd.trim(); // Get rid of any leading/trailing whitespace
  
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "Off") {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED on (HIGH is the voltage level)
  } else if (cmd == "On") {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
}

// Process a character's worth of serial input.
void processSerialInput(HardwareSerial& serial, const char* portName, String& inputBuffer) {
  if (!serial.available()) {
    // No new data waiting: bail out.
    return;
  }

  // Get the next character from the specified serial port
  char newChar = char(serial.read());

  // Figure out what that character means for us.
  if (isEndOfLineMarker(newChar)) {
    // OK, this character marks the end of a command.  Process what's in the
    // buffer, and then reset it so that we can start anew.

#ifdef ENABLE_DEBUGGING_OUTPUT
    // Show what we've received.
    //
    // Note that we convert any "end of line" marker characters into
    // a newline when we print them below, since we're not going to
    // put them in the inputBuffer.  (If you want to see them, you'd
    // need to add them to the end of the buffer before this, or do
    // something else along those lines.)
    Serial.print(portName);
    Serial.print(": ");
    Serial.println(inputBuffer);
#endif  // ENABLE_DEBUGGING_OUTPUT

    // Take action as directed.
    processCommand(inputBuffer);

    // Clear out the buffer, to start accumulating the next set of
    // data from the remote device.
    inputBuffer = "";
  } else {
    // Append the latest character to the input buffer.
    inputBuffer += newChar;
  }
}

void loop() {
  // Try reading from port 0.
  static String serialBuffer0;
  processSerialInput(Serial, "Serial 0", serialBuffer0);

#ifdef HAVE_HWSERIAL1
  // Try reading from port 0.
  static String serialBuffer1;
  processSerialInput(Serial1, "Serial 1", serialBuffer1);
#endif
}
