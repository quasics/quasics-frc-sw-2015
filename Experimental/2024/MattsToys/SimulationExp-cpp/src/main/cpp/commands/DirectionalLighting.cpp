// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DirectionalLighting.h"

#include "subsystems/IDrivebase.h"
#include "utils/BulletinBoard.h"

DirectionalLighting::DirectionalLighting(Lighting& lighting)
    : m_lighting(lighting) {
  AddRequirements(&m_lighting);
}

// Called repeatedly when this Command is scheduled to run
void DirectionalLighting::Execute() {
  const std::optional<std::string> directionOptional =
      BulletinBoard::getValue<std::string>(
          IDrivebase::BULLETIN_BOARD_DIRECTION_KEY);
  if (!directionOptional.has_value()) {
    m_lighting.setSolidStripColor(Lighting::ORANGE);
  } else {
    const auto direction = directionOptional.value();
    if (direction == IDrivebase::BULLETIN_BOARD_DIRECTION_FORWARD_VALUE) {
      m_lighting.setSolidStripColor(Lighting::GREEN);
    } else if (direction ==
               IDrivebase::BULLETIN_BOARD_DIRECTION_REVERSE_VALUE) {
      m_lighting.setSolidStripColor(Lighting::CYAN);
    } else if (direction ==
               IDrivebase::BULLETIN_BOARD_DIRECTION_TURNING_VALUE) {
      m_lighting.setSolidStripColor(Lighting::MAGENTA);
    } else if (direction ==
               IDrivebase::BULLETIN_BOARD_DIRECTION_STOPPED_VALUE) {
      m_lighting.setSolidStripColor(Lighting::WHITE);
    } else {
      // ?????
      m_lighting.setSolidStripColor(Lighting::BLACK);
    }
  }
}

// Called once the command ends or is interrupted.
void DirectionalLighting::End(bool interrupted) {
  m_lighting.setSolidStripColor(Lighting::WHITE);
}
