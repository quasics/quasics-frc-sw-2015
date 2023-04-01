// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ConfigSettings.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SignalRequestedPayload
    : public frc2::CommandHelper<frc2::CommandBase, SignalRequestedPayload> {
 public:
  SignalRequestedPayload(ConfigSettings* configSettings,
                         RequestedPayload payload)
      : m_configSettings(configSettings), m_payload(payload) {
  }

  void Initialize() override {
    if (m_configSettings != nullptr) {
      m_configSettings->requestedPayload = m_payload;
    }
  }

  bool IsFinished() override {
    return true;
  }

 private:
  ConfigSettings* const m_configSettings;
  const RequestedPayload m_payload;
};
