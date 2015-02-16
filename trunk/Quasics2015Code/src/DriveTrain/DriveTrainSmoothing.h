#ifndef DRIVE_TRAIN_SMOOTHING_H
#define DRIVE_TRAIN_SMOOTHING_H

extern const float joystickStep;

// Lets us figure out power settings we need to use as a translation of the
// joystick readings.
void TrimJoystickValuesToPower(
    bool useLinearScaling,
    float leftIn, float rightIn,
    float& leftOut, float& rightOut);


// "Convenience" function, which calls TrimJoystickValuesToPower(), passing
// true for the first parameter.
void TrimJoystickValuesToPowerWithLinearScaling(
    float leftIn, float rightIn,
    float& leftOut, float& rightOut);

// "Convenience" function, which calls TrimJoystickValuesToPower(), passing
// false for the first parameter.
void TrimJoystickValuesToPowerWithoutLinearScaling(
    float leftIn, float rightIn,
    float& leftOut, float& rightOut);

#endif  // DRIVE_TRAIN_SMOOTHING_H
