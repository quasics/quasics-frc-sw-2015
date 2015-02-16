#include "DriveTrainSmoothing.h"
#include <algorithm>
using namespace std;

//Data Pertaining to multiplier Table Start

// Data type for multiplier table
struct MultiplierDataType {
	float leftMultiplier;
	float rightMultiplier;
};
// Interval of data table
const float joystickStep = 0.05;
// Power Multiplier table. IMPORTANT: Keep scale consistent and update joysticStep as well
const MultiplierDataType multiplierData[] = { { 1, .97 }, // for joystick readings of -1.00
		{ .99, 1 },   // for joystick readings of -0.95
		{ .96, 1 },   // for joystick readings of -0.90
		{ 1, .96 },   // for joystick readings of -0.85
		// More values go here....
		{ 1, .96 }    // for joystick readings of +1.00
};
//Number of entries in multiplierData (Total Bytes / Bytes Per Entry)
const int sizeOfMultiplierDataTable = sizeof(multiplierData)
		/ sizeof(multiplierData[0]);

//Data Pertaining To Data Table End

/*Linear Interpolation Calculator
 *	Point below is (x0, y0), point above is (x1, y1)
 *	plots a point between them and evaluates that equation for x
 */
float LinearInterpolation(float x0, float y0, float x1, float y1, float x) {
	//To prevent a NAN, return y0
	if (x1 == x0) {
		return y0;
	}

	//Actual equation
	return (y0 + (y1 - y0) * (x - x0) / (x1 - x0));
}

/* Base function for trim calculation
 * Can either use stepped scaling or interpolated scaling, depending on the bool
 */
void TrimJoystickValuesToPower(bool useLinearScaling, float leftIn,
		float rightIn, float& leftOut, float& rightOut) {
	// bring inputs to between -1 and 1
	leftIn = min(max(leftIn, -1.0f), +1.0f);
	rightIn = min(max(rightIn, -1.0f), +1.0f);

	// Conversion from input to table
	const int indexOffset = sizeOfMultiplierDataTable / 2; // e.g., 41 / 2 ==> 20,
														   //since they are integers

	/* Scaling the values to an integer multiple of (roughly) half our table
	 size, so that we can convert them into an index into the table in the
	 next step.  (For example, -1 ==> -20, +1 ==> +20, etc.)

	 Note that this will always round down (e.g. .99 * 20 ==> 19.8 when scaled,
	 but we will round that down to 19, which gives us safe values for use in
	 figuring out the index).
	 */
	const int leftScaled = int(leftIn * indexOffset);
	const int rightScaled = int(rightIn * indexOffset);

	// OK, take the scaled size and do the conversion.
	const int leftIndex = leftScaled + indexOffset;
	const int rightIndex = rightScaled + indexOffset;

	//Set default multipliers (final multipliers if not using interpolation)
	float leftMultiplier = multiplierData[leftIndex].leftMultiplier;
	float rightMultiplier = multiplierData[rightIndex].rightMultiplier;

	if (useLinearScaling) {
		if (leftIndex == sizeOfMultiplierDataTable - 1) {
			//Since we can't pull a greater number, just return the default for max
		} else {
			// Define variables for interpolation
			const float x0 = leftScaled;	// previously defined floor
			const float y0 = multiplierData[leftIndex].leftMultiplier; //get Y value for x0
			const float x1 = leftScaled + joystickStep; //grab next value from data table
			const float y1 = multiplierData[leftIndex + 1].leftMultiplier; //evaluate for x1
			const float x = leftIn * indexOffset; //non-integer left scaled value

			leftMultiplier = LinearInterpolation(x0, y0, x1, y1, x); //run equation
		}

		if (rightIndex == sizeOfMultiplierDataTable - 1) {
			//Since we can't pull a greater number, just return the default for max
		} else {
			// Define variables for interpolation
			const float x0 = rightScaled;	// previously defined floor
			const float y0 = multiplierData[rightIndex].rightMultiplier; //get Y value for x0
			const float x1 = rightScaled + joystickStep; //grab next value from data table
			const float y1 = multiplierData[rightIndex + 1].rightMultiplier; //evaluate for x1
			const float x = rightIn * indexOffset; //non-integer left scaled value

			rightMultiplier = LinearInterpolation(x0, y0, x1, y1, x); //run equation
		}
	}

	//output trimmed values
	leftOut = leftMultiplier * leftIn;
	rightOut = rightMultiplier * rightIn;
}

//run stepped scaling
void TrimJoystickValuesToPowerWithoutLinearScaling(float leftIn, float rightIn,
		float& leftOut, float& rightOut) {
	return TrimJoystickValuesToPower(false, leftIn, rightIn, leftOut, rightOut);
}

//run interpolated scaling
void TrimJoystickValuesToPowerWithLinearScaling(float leftIn, float rightIn,
		float& leftOut, float& rightOut) {
	return TrimJoystickValuesToPower(true, leftIn, rightIn, leftOut, rightOut);
}
