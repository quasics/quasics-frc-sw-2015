// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class DebugSupport {
  public static class ThrottledLogger {
    private final int times;
    private int counter = 0;

    public ThrottledLogger(int times) {
      this.times = Math.max(times, 1);
    }

    public void log(String msg) {
      if (counter == 0) {
        System.err.println(msg);
      }
      counter = (counter + 1) % times;
    }

  }
}
