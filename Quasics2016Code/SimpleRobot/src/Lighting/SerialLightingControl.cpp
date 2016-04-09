#include "SerialLightingControl.h"

#define ENABLE_NOISY_LOGGING

std::unique_ptr<SerialPort> SerialLightingControl::serialPort;

SerialLightingControl::SerialLightingControl() {
    serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
}

void SerialLightingControl::SetState(State whichState) {
    std::string serialText = ";";
    switch (whichState) {
        case kRedTeam:
            serialText = "RedTeam;";
            break;
        case kBlueTeam:
            serialText = "BlueTeam;";
            break;
        case kDemo:
            serialText = "Demo;";
            break;
        default:
            serialText = "Error;";
            break;
    }

    const uint32_t bytesWritten = serialPort->Write(serialText,
                                  serialText.length());

#ifdef ENABLE_NOISY_LOGGING
    std::cout << "Wrote " << bytesWritten << " bytes of '" << serialText << "'"
              << std::endl;
#endif  // ENABLE_NOISY_LOGGING
}

void SerialLightingControl::SetMode(Mode whichMode) {
    std::string serialText = ";";
    switch (whichMode) {
        case kSolid:
            serialText = "Solid;";
            break;
        case kSlowBlinking:
            serialText = "SlowBlink;";
            break;
        case kMediumBlink:
            serialText = "MediumBlink;";
            break;
        case kBreathing:
            serialText = "Breathing;";
            break;
        default:
            serialText = "Error;";
            break;
    }

    const uint32_t bytesWritten = serialPort->Write(serialText,
                                  serialText.length());

#ifdef ENABLE_NOISY_LOGGING
    std::cout << "Wrote " << bytesWritten << " bytes of '" << serialText << "'"
              << std::endl;
#endif  // ENABLE_NOISY_LOGGING
}

void SerialLightingControl::SendHeartbeat() {
    std::string serialText = "Heartbeat;";
    // CODE_REVIEW(mjh): Should this start with a ";" (for safety), as you're
    // doing for the other commands?
    const uint32_t bytesWritten = serialPort->Write(serialText,
                                  serialText.length());

#ifdef ENABLE_NOISY_LOGGING
    std::cout << "Wrote " << bytesWritten << " bytes of '" << serialText << "'"
              << std::endl;
#endif  // ENABLE_NOISY_LOGGING
}

void SerialLightingControl::SendBatteryState(bool isLow) {
    std::string serialText = ";";
    if (isLow) {
        serialText = "LowBattery;";
    }
    else {
        serialText = "GoodBattery;";
    }

    const uint32_t bytesWritten = serialPort->Write(serialText,
                                  serialText.length());

#ifdef ENABLE_NOISY_LOGGING
    std::cout << "Wrote " << bytesWritten << " bytes of '" << serialText << "'"
              << std::endl;
#endif  // ENABLE_NOISY_LOGGING
}
