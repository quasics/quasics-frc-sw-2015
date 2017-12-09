/*
 * ControllerDefinitions.h
 *
 *  Created on: Dec 9, 2017
 *      Author: healym
 */

#ifndef SRC_CONTROLLERDEFINITIONS_H_
#define SRC_CONTROLLERDEFINITIONS_H_

// Defines shortcuts for each of the controllers we use, to make the code easier and more readable

//--------------------Logitech Gamepad-----------------------------------------------------------------
// Buttons
#define AButton 			2
#define BButton  			3
#define XButton  			4
#define YButton  			1
#define LeftShoulder  		5
#define RightShoulder  		6
#define LeftTrigger  		7
#define RightTrigger  		8
#define LeftStickPress  	11
#define RightStickPress  	12
#define StartButton  		9
#define SelectButton  		10

// Axes - Used with the "getRawAxis()" function to access the X/Y data for the individual sticks on
// the controller (e.g., for "tank drive" coding).
#define LeftYAxis  			1
#define LeftXAxis 			0
#define RightYAxis  		3
#define RightXAxis  		2

//--------------------X-Box Controller-----------------------------------------------------------------
// Buttons
#define XBox_ButtonA  		1
#define XBox_ButtonB  		2
#define XBox_ButtonX  		3
#define XBox_ButtonY  		4
#define XBox_LeftTrigger 	5

// Axes
#define XBox_RightXAxis  	2
#define XBox_RightYAxis  	3


#endif /* SRC_CONTROLLERDEFINITIONS_H_ */
