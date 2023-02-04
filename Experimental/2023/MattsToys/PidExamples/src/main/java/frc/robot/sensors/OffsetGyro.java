// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public class OffsetGyro {
  static private Gyro getGyroForFunction(DoubleSupplier supplier) {
    final double [] angleOffset = new double[1];
    return new FunctionalGyro(
      () -> {}, // close
      () -> {}, // calibrate
      () -> { angleOffset[0] = supplier.getAsDouble(); }, // reset
      () -> { return supplier.getAsDouble() - angleOffset[0]; }, // getAngle
      () -> { return 0; }  // getRate - not supported
      );
  }

  /**
   * @return
   */
  static public Gyro getYawGyro(final Pigeon2 pigeon) {
    return getGyroForFunction(pigeon::getYaw);
  }

  /**
   * @return
   */
  static public Gyro getPitchGyro(final Pigeon2 pigeon) {
    return getGyroForFunction(pigeon::getPitch);
  }

  /**
   * @return
   */
  static public Gyro getRollGyro(final Pigeon2 pigeon) {
    return getGyroForFunction(pigeon::getRoll);
  }
}
