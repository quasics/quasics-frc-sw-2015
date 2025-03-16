// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/** Misc. support functions for robot coding. */
public class SupportFunctions {
  /**
   * Returns true if the measure x is between a and b, inclusive. (Ordering of a
   * and b does not matter.)
   * 
   * Note that this should work for any set of 3 measurements of the same logical
   * type (e.g., 3 distances, angles, voltages, etc.).
   * 
   * @param x the value to check.
   * @param a one bound
   * @param b the other bound
   * 
   * @return true iff x is between a and b, inclusive.
   */
  public static <T extends Unit> boolean isBetween(Measure<T> x, Measure<T> a, Measure<T> b) {
    return (x.gte(a) && x.lte(b)) ||
        (x.gte(b) && x.lte(a));
  }

}
