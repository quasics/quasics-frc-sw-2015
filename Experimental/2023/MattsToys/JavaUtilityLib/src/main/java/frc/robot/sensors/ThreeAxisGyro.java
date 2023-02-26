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
  public class SingleAxisWrapper implements Gyro {
    /** Source for reporting the angle. */
    final Supplier<Double> m_angleSupplier;

    /** Used by "reset()" to establish a new baseline for the angle. */
    double m_calibrationOffset = 0;

    SingleAxisWrapper(Supplier<Double> supplier) {
      m_angleSupplier = supplier;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void calibrate() {}

    @Override
    public void reset() {
      m_calibrationOffset = m_angleSupplier.get();
    }

    @Override
    public double getAngle() {
      return m_angleSupplier.get() - m_calibrationOffset;
    }

    @Override
    public double getRate() {
      throw new UnsupportedOperationException("getRate() isn't supported");
    }
  }
}
