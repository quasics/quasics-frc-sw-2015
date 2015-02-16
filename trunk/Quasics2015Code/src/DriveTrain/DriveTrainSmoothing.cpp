#include "DriveTrainSmoothing.h"
#include <algorithm>

using namespace std;

/// "Power multipliers", used to convert a pair of values used to convert a
/// joystick reading into the (desired) output power setting for the
/// corresponding motor.
struct JoystickToPowerMultipliers {
  float leftMultiplier;
  float rightMultiplier;
};

const float joystickStep = 0.5;

/// Table of "power multipliers" for use by the TranslateValues() function.
/// The values are spaced out by "joystickStep" increments on the joystick readings, so
/// that the first represents the multipliers to be used for joystick readings
/// of -1.00, the second for readings of -0.95, etc., on up to readings of
/// +1.00.
const JoystickToPowerMultipliers joystickToPowerMultipliers[] = {
  { 1, .97 },   // for joystick readings of -1.00
  { .99, 1 },   // for joystick readings of -0.95
  { .96, 1 },   // for joystick readings of -0.90
  { 1, .96 },   // for joystick readings of -0.85
  // More values go here....
  { 1, .96 }    // for joystick readings of +1.00
};

const int sizeOfJoystickToPowerMultipliersTable =
    sizeof(joystickToPowerMultipliers) / sizeof(joystickToPowerMultipliers[0]);

float LinearInterpolation(
                          float x0, float y0,   // first point on curve (x is input, y is result)
                          float x1, float y1,   // second point on curve (ditto)
                          float x)              // input being evaluated for the function
{
  if ( x1 == x0 ) {
    // This would be bad: either a single point is being used, or else we
    // are going to wind up with infinite slope.  Either way, it's bad, so
    // we will "chicken out" and just return y0.
    return y0;
  }
  
  return (y0 + (y1 - y0) * (x - x0) / (x1 - x0));
}

/// Note that this funtion assumes that joystick readings go from -1 to +1, and
/// that the corresponding entries in the table will be spaced at intervals
/// of 0.05 (so that there are 41 readings in the table).
///
/// This function could be improved to perform linear interpolation to compute
/// approximate values for joystick readings between points used for the
/// table, so that the drive responds in a more analog fashion to changes on
/// the joystick, rather than "jumping" from one point to another.  (See
/// https://en.wikipedia.org/wiki/Linear_interpolation for a discussion of
/// linear interpolation, including the formula and how to use it.)
void TrimJoystickValuesToPower(bool useLinearScaling,
                               float leftIn, float rightIn,
                               float& leftOut, float& rightOut) {
  // "Sanitize" our inputs, making sure that they are always between -1.0 and
  // +1.0, inclusive.
  leftIn = min(max(leftIn, -1.0f), +1.0f);
  rightIn = min(max(rightIn, -1.0f), +1.0f);
  
  const int indexOffset =
  sizeOfJoystickToPowerMultipliersTable / 2;  // e.g., 41 / 2 ==> 20,
  // since they are integers
  
  // Scaling the values to an integer multiple of (roughly) half our table
  // size, so that we can convert them into an index into the table in the
  // next step.  (For example, -1 ==> -20, +1 ==> +20, etc.)
  //
  // Note that this will always round down (e.g. .99 * 20 ==> 19.8 when scaled,
  // but we will round that down to 19, which gives us safe values for use in
  // figuring out the index).
  const int leftScaled = int(leftIn * indexOffset);
  const int rightScaled = int(rightIn * indexOffset);
  
  // OK, take the scaled size and do the conversion.
  const int leftIndex = leftScaled + indexOffset;
  const int rightIndex = rightScaled + indexOffset;
  
  float leftMultiplier = joystickToPowerMultipliers[leftIndex].leftMultiplier;
  float rightMultiplier = joystickToPowerMultipliers[rightIndex].rightMultiplier;
  
  if ( useLinearScaling ) {
    if ( leftIndex == sizeOfJoystickToPowerMultipliersTable - 1 ) {
      // Last value in the table: can't plot any further out, so we'll stick
      // with the value pulled when we initialized the leftMultiplier.
    } else {
      const float x0 = leftScaled;
      const float y0 = joystickToPowerMultipliers[leftIndex].leftMultiplier;
      const float x1 = leftScaled + indexOffset;
      const float y1 = joystickToPowerMultipliers[leftIndex+1].leftMultiplier;
      const float x = leftIn * indexOffset;
      
      leftMultiplier = LinearInterpolation(x0, y0, x1, y1, x);
    }

    if ( rightIndex == sizeOfJoystickToPowerMultipliersTable - 1 ) {
      // Last value in the table: can't plot any further out, so we'll stick
      // with the value pulled when we initialized the rightMultiplier.
    } else {
      const float x0 = rightScaled;
      const float y0 = joystickToPowerMultipliers[rightIndex].rightMultiplier;
      const float x1 = rightScaled + indexOffset;
      const float y1 = joystickToPowerMultipliers[rightIndex+1].rightMultiplier;
      const float x = rightIn * indexOffset;
      
      rightMultiplier = LinearInterpolation(x0, y0, x1, y1, x);
    }
  }

  leftOut = leftMultiplier * leftIn;
  rightOut = rightMultiplier * rightIn;
}

void TrimJoystickValuesToPowerWithoutLinearScaling(
    float leftIn, float rightIn,
    float& leftOut, float& rightOut) {
  return TrimJoystickValuesToPower(false, leftIn, rightIn, leftOut, rightOut);
}

void TrimJoystickValuesToPowerWithLinearScaling(
    float leftIn, float rightIn,
    float& leftOut, float& rightOut) {
  return TrimJoystickValuesToPower(true, leftIn, rightIn, leftOut, rightOut);
}
