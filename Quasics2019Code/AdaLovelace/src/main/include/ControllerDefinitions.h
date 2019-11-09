#ifndef SRC_CONTROLLERDEFINITIONS_H_
#define SRC_CONTROLLERDEFINITIONS_H_

// Defines shortcuts for each of the controllers we use, to make the code easier
// and more readable

//--------------------Logitech
//Gamepad-----------------------------------------------------------------
// Buttons
#define LogitechGamePad_AButton 2
#define LogitechGamePad_BButton 3
#define LogitechGamePad_XButton 4
#define LogitechGamePad_YButton 1
#define LogitechGamePad_LeftShoulder 5
#define LogitechGamePad_RightShoulder 6
#define LogitechGamePad_LeftTrigger 7
#define LogitechGamePad_RightTrigger 8
#define LogitechGamePad_LeftStickPress 11
#define LogitechGamePad_RightStickPress 12
#define LogitechGamePad_StartButton 9
#define LogitechGamePad_SelectButton 10

// Axes - Used with the "getRawAxis()" function to access the data for the
// individual sticks on the controller (e.g., for "tank drive" coding).
#define LogitechGamePad_LeftXAxis 2
#define LogitechGamePad_LeftYAxis 5
#define LogitechGamePad_RightXAxis 0
#define LogitechGamePad_RightYAxis 1

//--------------------X-Box
//Controller-----------------------------------------------------------------
// Buttons
#define XBox_ButtonA 1
#define XBox_ButtonB 2
#define XBox_ButtonX 3
#define XBox_ButtonY 4
#define XBox_LeftButton 5
#define XBox_RightButton 6

// Axes
#define XBox_LeftXAxis 0
#define XBox_LeftYAxis 1
#define XBox_RightXAxis 2
#define XBox_RightYAxis 5

#define XBox_LeftTrigger 2
#define XBox_RightTrigger 3

#endif /* SRC_CONTROLLERDEFINITIONS_H_ */
