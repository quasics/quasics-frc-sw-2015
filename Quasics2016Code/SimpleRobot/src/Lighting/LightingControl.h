/*
 * LightingControl.h
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#ifndef SRC_LIGHTING_LIGHTINGCONTROL_H_
#define SRC_LIGHTING_LIGHTINGCONTROL_H_

#include <WPILib.h>
#include <iostream>

class LightingControl {
public:
    LightingControl();
    virtual ~LightingControl() {}

    /** Main logic control goes here, allowing this function to handle
     * common decision-making.  Actual communications with the Arduino
     * is handled by the protected functions (below), and will be
     * implemented in derived classes, allowing the decision of the
     * communications channel being used (serial port, I2C, etc.) to
     * be deferred.
     */
    void LightingUpkeep();

// Stuff dealing with actual communications with the Arduino,
// which will be *really* implemented in derived classes.
protected:
    enum State {
        kRedTeam = 0, kBlueTeam = 1, kDemo = 2, kError = 3
    };
    enum Mode {
        kSolid = 0,
        kSlowBlinking = 1,
        kMediumBlink = 2,
        kBreathing = 3,
        kErrorMode = 4
    };

    // CODE_REVIEW(mjh): Should this set of functions be "const" methods?
    // CODE_REVIEW(mjh): Naming is inconsistent.  Why are some "set", and
    // others are "send"?
    virtual void SetState(State whichState) = 0;
    virtual void SetMode(Mode whichMode) = 0;
    virtual void SendHeartbeat() = 0;
    virtual void SendBatteryState(bool isLow) = 0;

// Utility functions (isolating some decisions)
private:
    State ComputeCurrentState() const;
    Mode ComputeCurrentMode() const;

private:
    // State used in the "upkeep" function.
    int lastBatterySwitch;
    int lightingTimer;
    bool previousBatteryIsLow;
    State previousState;
    Mode previousMode;

    friend std::ostream& operator<<(std::ostream& os, State s);
    friend std::ostream& operator<<(std::ostream& os, Mode m);
};

// A trivial implementation of the LightingControl interface, which
// does nothing to try to change lights.
class NullLightingControl : public LightingControl {
protected:
    virtual void SetState(State whichState) {}
    virtual void SetMode(Mode whichMode) {}
    virtual void SendHeartbeat() {}
    virtual void SendBatteryState(bool isLow) {}
};

#endif /* SRC_LIGHTING_LIGHTINGCONTROL_H_ */
