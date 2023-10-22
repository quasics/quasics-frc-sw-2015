/**
 * @file Precompiler controls, being used to turn blocks of code (e.g.,
 * subsystems, motors, etc.) on/off.
 */
#pragma once

#undef ENABLE_PHOTONLIB  // Disabled for Beta 1 (for now)

//
// Debugging support
//

#undef SHOW_SUBSYSTEMS_ON_DASHBOARD

//
// Operator interface configuration
//

#undef DUAL_LOGITECH_CONTROLLERS

//
// High-level control of functionality in subsystems.
//

#undef ENABLE_MATCH_PLAY_LIGHTING  // Disabled for Beta 1 (for now)

// Controlling which gyro is enabled for the robot.
#define ENABLE_AD_GYRO  // Enabled for Beta 1 (for now)
#undef ENABLE_PIGEON    // Disabled for Beta 1 (for now)

// If defined, enables the use of additional limit switches with the intake (2
// on each side: 1 for extended, 1 for retracted).
#undef ENABLE_EXPANDED_INTAKE_LIMIT_SWITCHES

// If defined, enables the motors on the roller-based intake.
#define ENABLE_ROLLER_INTAKE_MOTORS

// If defined, enables the motors on intake deployment/retraction
// subsystem.
#define ENABLE_INTAKE_DEPLOYMENT_MOTORS

// If defined, enables attempting to detect when the intake is running up
// against a hard stop while extending/retracting.
#undef ENABLE_INTAKE_HARD_STOP_DETECTION

//
// High-level control of options in auto mode.
//

// If defined, enables the roller during auto mode.
//
// Note that it's OK for this to be disabled when the roller is enabled,
// since we might not want to actually have things happen.
#define USING_ROLLER_FOR_AUTO_INTAKE

#define USING_GLADYS_TRAJECTORY_CONSTANTS
#undef USING_SALLY_TRAJECTORY_CONSTANTS
#undef USING_MAE_TRAJECTORY_CONSTANTS