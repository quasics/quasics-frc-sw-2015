/**
 * @file Precompiler controls, being used to turn blocks of code (e.g.,
 * subsystems, motors, etc.) on/off.
 */
#pragma once

//
// High-level control of functionality in subsystems.
//

#undef ENABLE_MATCH_PLAY_LIGHTING

// Controlling which gyro is enabled for the robot.
#undef ENABLE_AD_GYRO
#define ENABLE_PIGEON

// If defined, enables the use of additional limit switches with the intake (2
// on each side: 1 for extended, 1 for retracted).
#undef ENABLE_EXPANDED_INTAKE_LIMIT_SWITCHES

// If defined, enables the motors on the clamp subsystem.
#undef ENABLE_CLAMP_MOTORS

// If defined, enables the motors on the roller-based intake.
#define ENABLE_ROLLER_INTAKE_MOTORS

// If defined, enables the motors on intake deployment/retraction
// subsystem.
#define ENABLE_INTAKE_DEPLOYMENT_MOTORS

// If defined, enables the use of an encoder on the floor ejection
// subsystem.
#undef ENABLE_FLOOR_EJECTION_ENCODER

// If defined, enables attempting to detect when the intake is running up
// against a hard stop while extending/retracting.
#undef ENABLE_INTAKE_HARD_STOP_DETECTION

//
// High-level control of options in auto mode.
//

// Note that it's OK for this to be disabled when the clamp is enabled,
// since we might not want to actually have things happen.
#undef USING_CLAMP_FOR_AUTO_INTAKE

// If defined, enables the roller during auto mode.
//
// Note that it's OK for this to be disabled when the roller is enabled,
// since we might not want to actually have things happen.
#define USING_ROLLER_FOR_AUTO_INTAKE
