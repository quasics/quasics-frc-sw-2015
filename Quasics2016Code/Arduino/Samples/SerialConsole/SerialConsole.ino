/*
 "Console" application for a device connected to Serial1

 Receives from the main serial port, sends to Serial1.
 Receives from serial port 1, sends to the main serial (Serial 0).

 This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc

 The circuit:
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:
 */


void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("SerialConsole now running....");
}

void loop() {
  // read from port 1, send to port 0:
  while (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  while (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }
}
