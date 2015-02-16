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
const MultiplierDataType multiplierData[] = {
		{ 1 , .97 },		//-1.00
		{ .91 , 1 },		//-0.95
		{ .99 , 1 },		//-0.90
		{ .90 , 1 },		//-0.85
		{ .97 , 1 },		//-0.80
		{ .91 , 1 },		//-0.75
		{ .96 , 1 },		//-0.70
		{ .90 , 1 },		//-0.65
		{ .94 , 1 },		//-0.60
		{ .88 , 1 },		//-0.55
		{ .93 , 1 },		//-0.50
		{ .85 , 1 },		//-0.45
		{ .89 , 1 },		//-0.40
		{ .80 , 1 },		//-0.35
		{ .83 , 1 },		//-0.30
		{ .70 , 1 },		//-0.25
		{ .71 , 1 },		//-0.20
		{ .43 , 1 },		//-0.15
		{ 0 , 0 },			//-0.10
		{ 0 , 0 },			//-0.05
		{ 0 , 0 },			//+0.00
		{ 0 , 0 },			//+0.05
		{ 0 , 0 },			//+0.10
		{ .90 , 1 },		//+0.15
		{ 1 , .96 },		//+0.20
		{ 1 , .95 },		//+0.25
		{ 1 , .93 },		//+0.30
		{ 1 , .99 },		//+0.35
		{ 1 , .94 },		//+0.40
		{ 1 , .99 },		//+0.45
		{ 1 , .93 },		//+0.50
		{ 1 , .98 },		//+0.55
		{ 1 , .93 },		//+0.60
		{ 1 , .97 },		//+0.65
		{ 1 , .92 },		//+0.70
		{ 1 , .96 },		//+0.75
		{ 1 , .91 },		//+0.80
		{ 1 , .96 },		//+0.85
		{ 1 , .92 },		//+0.90
		{ 1 , .96 },		//+0.95
		{ 1 , .96 }			//+1.00
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
