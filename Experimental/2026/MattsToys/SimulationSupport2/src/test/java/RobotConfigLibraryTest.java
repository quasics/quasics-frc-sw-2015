// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.util.RobotConfigLibrary;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotConfigLibraryTest {
  @BeforeEach
  void setup() {
  }

  @AfterEach
  void shutdown() throws Exception {
  }

  @Test
  void mapCompleteTest() {
    // Size of the map should be match the number of known robots.
    assertEquals(RobotConfigLibrary.Robot.values().length, RobotConfigLibrary.getRobotConfigMap().size());

    // No robot should have a "null" configuration.
    for (var robot : RobotConfigLibrary.Robot.values()) {
      assertNotEquals(null, RobotConfigLibrary.getConfig(robot));
    }
  }
}