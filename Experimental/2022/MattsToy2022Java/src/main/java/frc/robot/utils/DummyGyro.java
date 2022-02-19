// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public class DummyGyro implements Gyro {
  public DummyGyro() {
    System.err.println("********\n" +
        "* Creating dummy gyro: all headings from this will be 0!!!!" +
        "********\n");
  }

  @Override
  public void close() throws Exception {
  }

  @Override
  public void calibrate() {
  }

  @Override
  public void reset() {
  }

  @Override
  public double getAngle() {
    return 0;
  }

  @Override
  public double getRate() {
    return 0;
  }
}
