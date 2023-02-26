// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import java.util.function.Supplier;

/** Add your docs here. */
public abstract interface ThreeAxisGyro {
  /** Returns a Gyro object that can track rotation on the X axis (roll). */
  public abstract Gyro getRollGyro();

  /** Returns a Gyro object that can track rotation on the Y axis (pitch). */
  public abstract Gyro getPitchGyro();

  /** Returns a Gyro object that can track rotation on the Z axis (yaw). */
  public abstract Gyro getYawGyro();

  /** Simple wrapper class to be help implement the interface. */
  public class SingleAxisWrapper extends OffsetGyro {
    public SingleAxisWrapper(
        final Supplier<Double> angleSupplier, final Supplier<Double> rateSupplier) {
      super(
          new SimulatedGyro(
              SimulatedGyro.TRIVIAL_RUNNABLE,
              SimulatedGyro.TRIVIAL_RUNNABLE,
              SimulatedGyro.TRIVIAL_RUNNABLE,
              angleSupplier,
              () -> {
                if (rateSupplier == null) {
                  throw new UnsupportedOperationException("Can't supply rate");
                } else {
                  return rateSupplier.get();
                }
              }));
    }

    public SingleAxisWrapper(Supplier<Double> angleSupplier) {
      this(angleSupplier, null);
    }
  }
}
