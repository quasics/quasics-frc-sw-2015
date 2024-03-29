// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * A completely fake gyro, which always reports "no movement".
 */
public class DummyGyro extends SimulatedGyro {
  public DummyGyro() {
    super(
        /* close */ () -> {
        },
        /* calibrate */ () -> {
        },
        /* reset */ () -> {
        },
        /* getAngle */ () -> {
          return 0;
        },
        /* getRate */ () -> {
          return 0;
        });

    System.err.println("********\n" +
        "* Creating dummy gyro: all headings from this will be 0!!!!" +
        "********\n");
  }
}
