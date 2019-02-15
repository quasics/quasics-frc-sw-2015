#ifndef CONFIGURATION_FLAGS_H
#define CONFIGURATION_FLAGS_H

// If DEFINED, don't enable default commands for the Elevator/Lifter subsystems.
//
// This is intended for use during integration testing.
#define DISABLE_ELEVATOR_DEFAULTS

// If DEFINED, disable (ignore) the limit switches on the Elevator/Lifter
// subsystems.
//
// This is intended for use when the limit switches aren't installed.
#define DISABLE_ELEVATOR_LIMIT_SWITCHES

#endif  // CONFIGURATION_FLAGS_H
