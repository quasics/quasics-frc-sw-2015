#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

enum class RequestedPayload { eNothing, eCubes, eCones };

struct ConfigSettings {
  bool intakingCubes = true;
  RequestedPayload requestedPayload = RequestedPayload::eNothing;
};

#endif  // CONFIG_SETTINGS_H
