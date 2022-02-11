// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * General interface for things that will manipulate motor speeds (e.g., as
 * specified by the driver joysticks).
 */
public interface SpeedModifier {
  /**
   * Takes the specified input %age, and applies some type of modifier to it
   * (e.g., for "dead band" handling, or speed caps, etc.).
   * 
   * @param inputPercentage input %age of motor speed, to be adjusted
   * @return the modified value for the speed to be applied to a motor
   */
  double adjustSpeed(double inputPercentage);
}
