/*
 * "Console" application for a device connected to Serial1, which includes
 * some simple buffering of the I/O.
 *
 * Receives from the main serial port, sends to Serial1.
 * Receives from serial port 1, sends to the main serial (Serial 0).
 *
 * Note: This example works *only* with boards with more than one serial
 * port, like Arduino Mega, Due, Zero etc.  (It will fail to compile for
 * boards with only one serial port.)
 *
 * The circuit:
 *   * Any serial device attached to Serial port 1
 *   * Serial monitor open on Serial port 0:
 */


void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("SerialConsole now running....");
}

String inputBuffer;
String outputBuffer;

#define ENABLE_DEBUGGING_OUTPUT

bool isEndOfLineMarker(char c) {
  return c == '\n' || c == ';';
}

void loop() {
  // read from port 1, buffer for display on port 0
  if (Serial1.available()) {
    char input = char(Serial1.read());
    if (isEndOfLineMarker(input)) {
      // Show what we've received.
      //
      // Note that we convert any "end of line" marker characters into
      // a newline when we print them below, since we're not going to
      // put them in the inputBuffer.  (If you want to see them, you'd
      // need to add them to the end of the buffer before this, or do
      // something else along those lines.)
      Serial.println("Received: " + inputBuffer);

      // Clear out the buffer, to start accumulating the next set of
      // data from the remote device.
      inputBuffer = "";
    } else {
      // Append the latest character to the input buffer.
      inputBuffer += input;
    }
  }

  // read from port 0, buffer and send out port 1
  if (Serial.available()) {
    char output = char(Serial.read());

    // Append the latest character to the output buffer.
    outputBuffer += output;

    // If we've reached an "end of line marker", then send the whole set
    // of accumulated data.  Note that this *will* include the end of line
    // marker, so that it can be seen/acted on by the other side.
    if (isEndOfLineMarker(output)) {
#ifdef ENABLE_DEBUGGING_OUTPUT
      // We'll also re-display it in the serial monitor
      Serial.print("Sending: " + outputBuffer);
      if (output != '\n') {
        Serial.println();
      }
#endif

      // Send the data to the remote device.  (We could probably do this
      // with "Serial1.print()", but that does internal buffering, too, which
      // we'd like to avoid.)
      for(int i = 0; i < outputBuffer.length(); ++i) {
        Serial1.write(outputBuffer[i]);
      }

      // Clear out the buffer, to start accumulating the next set of
      // data to be sent to the remote device.
      outputBuffer = "";
    }
  }
}
