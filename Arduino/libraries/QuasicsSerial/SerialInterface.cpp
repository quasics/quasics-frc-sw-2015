#include "serialInterface.h"

SerialInterface::SerialInterface(uint32_t baud) {
	Serial1.begin(baud);
	serialIn = "";
}

void SerialInterface::SerialRead(bool& stringChanged, String& output) {
	if (Serial1.available() > 0) {
		stringChanged = true;
		while (Serial1.available() > 0) {
			serialIn += char(Serial1.read());
			delayMicroseconds(100);
		}
		output = serialIn;
		serialIn = "";
	} else {
		stringChanged = false;
		output = "";
	}
}

void SerialInterface::SerialWrite (String toWrite){
	Serial1.print(toWrite);
}
