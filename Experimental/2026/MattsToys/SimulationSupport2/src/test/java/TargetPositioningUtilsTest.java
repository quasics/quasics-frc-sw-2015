// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.games.rebuilt.TargetPositioningUtils;

public class TargetPositioningUtilsTest {
  @BeforeEach
  void setup() {
  }

  @AfterEach
  void shutdown() throws Exception {
  }

  @Test
  void hubCenterCalculatorTest() {
    // Checking for exact equality with floating-point numbers is hard. Instead,
    // check "within tolerance".
    final double TOLERANCE_INCHES = .25;

    // Check data for Blue alliance (x, y) within tolerance
    assertEquals(
        182.11,
        TargetPositioningUtils.getHubCenterLocation(Alliance.Blue).getMeasureX().in(Inches), TOLERANCE_INCHES);
    assertEquals(
        158.84,
        TargetPositioningUtils.getHubCenterLocation(Alliance.Blue).getMeasureY().in(Inches), TOLERANCE_INCHES);

    // Check data for Red alliance (x, y) within tolerance
    assertEquals(
        469.11,
        TargetPositioningUtils.getHubCenterLocation(Alliance.Red).getMeasureX().in(Inches), TOLERANCE_INCHES);
    assertEquals(
        158.84,
        TargetPositioningUtils.getHubCenterLocation(Alliance.Red).getMeasureY().in(Inches), TOLERANCE_INCHES);
  }
}
