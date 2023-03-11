package frc.robot.utils;

public final class MathUtils {
  /**
   * Determine whether two numbers are "approximately equal" by seeing if they are within a certain
   * "tolerance percentage," with `tolerancePercentage` given as a percentage (such as 10.0 meaning
   * "10%").
   *
   * <p>This is also meant to help account for the fact that comparison of floating-point numbers
   * for precise equality is usually unsafe.
   *
   * @param tolerancePercentage 0.01 = 1%, 0.025 = 2.5%, etc.
   * @see https://alvinalexander.com/source-code/java-approximately-equals-method-function/
   */
  public static boolean approximatelyEqual(
      double desiredValue, double actualValue, double tolerancePercentage) {
    double diff = Math.abs(desiredValue - actualValue);
    double tolerance = desiredValue * tolerancePercentage;
    return diff < tolerance;
  }
}
