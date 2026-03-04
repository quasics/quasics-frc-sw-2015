// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.robot.util.LinearInterpolator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LinearInterpolatorTest {
  LinearInterpolator m_interpolator;
  /*
   * // Example lookup table: distance (inches) -> velocity (RPM)
   * // Real-world values require physical testing
   * m_interpolator.addDataPoint(100.0, 3000.0); // At 100 inches, use 3000 RPM
   * m_interpolator.addDataPoint(200.0, 4500.0); // At 200 inches, use 4500 RPM
   * m_interpolator.addDataPoint(300.0, 5500.0); // At 300 inches, use 5500 RPM
   */

  @BeforeEach
  void setup() {
    // Do any common pre-test setup in here (e.g., creating a subsystem,
    // initializing the HAL, etc.)
    m_interpolator = new LinearInterpolator();
  }

  @AfterEach
  void shutdown() throws Exception {
    // Do any common post-test cleanup in here (e.g., calling "close()" on
    // subsystem objects)
  }

  @Test
  void belowFloorTest() {
    m_interpolator.addDataPoint(100.0, 3000.0);
    m_interpolator.addDataPoint(200.0, 4500.0);
    m_interpolator.addDataPoint(300.0, 5500.0);
    assertEquals(3000, m_interpolator.getTargetApproximationForKey(99));
  }

  @Test
  void aboveCeilingTest() {
    m_interpolator.addDataPoint(100.0, 3000.0);
    m_interpolator.addDataPoint(200.0, 4500.0);
    m_interpolator.addDataPoint(300.0, 5500.0);
    assertEquals(5500, m_interpolator.getTargetApproximationForKey(301));
  }

  @Test
  void atDataPointsTest() {
    m_interpolator.addDataPoint(100.0, 3000.0);
    m_interpolator.addDataPoint(200.0, 4500.0);
    m_interpolator.addDataPoint(300.0, 5500.0);
    assertEquals(3000, m_interpolator.getTargetApproximationForKey(100));
    assertEquals(4500, m_interpolator.getTargetApproximationForKey(200));
    assertEquals(5500, m_interpolator.getTargetApproximationForKey(300));
  }

  @Test
  void betweenDataPointsTest() {
    m_interpolator.addDataPoint(100.0, 3000.0);
    m_interpolator.addDataPoint(200.0, 4500.0);
    m_interpolator.addDataPoint(300.0, 5500.0);
    // Mid-way between the points should be a straight average of the values on
    // either side.
    assertEquals(3750, m_interpolator.getTargetApproximationForKey(150));
    assertEquals(5000, m_interpolator.getTargetApproximationForKey(250));
  }

  @Test
  void emptyTableTest() {
    assertThrows(IllegalStateException.class, () -> {
      m_interpolator.getTargetApproximationForKey(0);
    });
  }
}
