/**
 * @file Precompiler controls, being used to turn blocks of code (e.g.,
 * subsystems, motors, etc.) on/off.
 */
#pragma once

#undef ENABLE_CLAMP_MOTORS

#define ENABLE_ROLLER_INTAKE_MOTORS

#define ENABLE_INTAKE_DEPLOYMENT_MOTORS

// Controlling which gyro is enabled for the robot.
#undef ENABLE_AD_GYRO
#define ENABLE_PIGEON

//
// High-level control of options in auto mode.
//

// Note that it's OK for this to be disabled when the clamp is enabled,
// since we might not want to actually have things happen.
#undef USING_CLAMP_FOR_AUTO_INTAKE

// Note that it's OK for this to be disabled when the roller is enabled,
// since we might not want to actually have things happen.
#undef USING_ROLLER_FOR_AUTO_INTAKE