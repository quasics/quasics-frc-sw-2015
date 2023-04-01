#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

/** Types of things we can ask the human player to feed the robot. */
enum class RequestedPayload { eNothing, eCubes, eCones };

/** Structure used to hold data used across commands. */
struct ConfigSettings {
  /** If false, signals that "switch mode" is engaged for driving. */
  volatile bool normalDriveEngaged = true;

  /** Used to signal intake speed (based on target game piece type). */
  volatile bool intakingCubes = true;

  /** Used to signal human player what should be loaded. */
  volatile RequestedPayload requestedPayload = RequestedPayload::eNothing;
};

#endif  // CONFIG_SETTINGS_H
