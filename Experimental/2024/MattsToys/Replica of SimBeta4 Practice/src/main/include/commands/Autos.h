// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/IDrivebase.h"

namespace autos {
  frc2::CommandPtr ExampleAuto(IDrivebase* subsystem);
}  // namespace autos
