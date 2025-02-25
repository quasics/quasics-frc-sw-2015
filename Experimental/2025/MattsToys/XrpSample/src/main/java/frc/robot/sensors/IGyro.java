// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import java.util.function.Supplier;

/**
 * This defines a "wrapper" type that can be used to let any arbitrary "Gyro" or
 * ALU object be used in a common way, even if they don't share a common base
 * class. (This wrapper provides a pretty basic view of gyros/ALUs, but that's
 * the point; I'm just looking for a way to use all kinds of an ALU as though
 * there are a common/single kind of object.)
 *
 * As context:
 * <ul>
 * <li>
 * Prior to the 2024 WPI tools, there was a common "Gyro" interface that
 * many gyros/ALUs implemented. This allowed them to be used
 * semi-interchangeably by code (e.g., if you're writing code that will
 * run on robots that might not always have the same kind of ALU installed,
 * such as a Pigeon2 over CAN on one drive base and an ADI ALU connected via
 * SPI on another). This interface wasn't supported by all ALUs, but it was
 * at least reasonably common.
 * </li>
 * <li>
 * As a part of the updates for the 2024 WPI tools, this common interface
 * was "deprecated" (meanining that it's going away in the future), and
 * the WPI team removed its use immediately from the various classes for
 * gyros/IMUs in the WPILib. (And other folks providing vendordeps for
 * specific hardware are generally following suit.)
 * </li>
 * <li>
 * These means gthat we are now effectively left without a common base type,
 * which means that it's *much* harder to write one piece of code that will
 * work with multiple ALUs, which is a real problem.
 * </li>
 * </ul>
 *
 * So, I'm defining my own (minimal) "wrapper" interface, which can be used to
 * adapt any arbitrary gyro/ALU to a common type, along with some functions to
 * help encapsulate specific examples "real" gyro classes with the wrapper.
 *
 * @see https://refactoring.guru/design-patterns/decorator
 * @see https://en.wikipedia.org/wiki/Adapter_pattern
 */
public interface IGyro {
  /** Tells the gyro to perform any calibration processing (e.g., on power-up). */
  void calibrate();

  /** Returns the heading of the robot in degrees. */
  Angle getAngle();

  /** Returns the rate of rotation of the gyro. */
  AngularVelocity getRate();

  /** Returns the heading of the robot as a Rotation2d. */
  Rotation2d getRotation2d();

  /**
   * Tells the gyro to reset its data, making the current heading the new "0"
   * value.
   */
  void reset();

  /**
   * A helper class that implements IGyro, and makes it easier to wrap arbitrary
   * types within this interface.
   */
  public class FunctionalGyro implements IGyro {
    private final Runnable m_calibrator;
    private final Supplier<Angle> m_angleSupplier;
    private final Supplier<AngularVelocity> m_rateSupplier;
    private final Supplier<Rotation2d> m_rotationSupplier;
    private final Runnable m_resetter;

    /**
     * Constructor, accepting an input function for each of the supported
     * operations.
     */
    FunctionalGyro(Runnable calibrator, Supplier<Angle> angleSupplier,
        Supplier<AngularVelocity> rateSupplier, Supplier<Rotation2d> rotationSupplier,
        Runnable resetter) {
      m_calibrator = calibrator;
      m_angleSupplier = angleSupplier;
      m_rateSupplier = rateSupplier;
      m_rotationSupplier = rotationSupplier;
      m_resetter = resetter;
    }

    @Override
    public void calibrate() {
      m_calibrator.run();
    }

    @Override
    public Angle getAngle() {
      return m_angleSupplier.get();
    }

    @Override
    public AngularVelocity getRate() {
      return m_rateSupplier.get();
    }

    @Override
    public Rotation2d getRotation2d() {
      return m_rotationSupplier.get();
    }

    @Override
    public void reset() {
      m_resetter.run();
    }
  }

  /**
   * @param g gyro to be put in a read-only wrapper
   * @return read-only version of an IGyro (disabling reset functionality
   */
  static IGyro readOnlyGyro(IGyro g) {
    return new FunctionalGyro(g::calibrate, g::getAngle, g::getRate, g::getRotation2d, () -> {});
  }

  /** Helper function to wrap the AnalogGyro type from WPILib. */
  static IGyro wrapGyro(AnalogGyro g) {
    final Runnable calibrator = () -> {
      g.calibrate();
    };
    final Supplier<Angle> angleSupplier = () -> Degrees.of(g.getAngle());
    final Supplier<AngularVelocity> rateSupplier = () -> DegreesPerSecond.of(g.getRate());
    final Supplier<Rotation2d> rotationSupplier = () -> g.getRotation2d();
    final Runnable resetter = () -> {
      g.reset();
    };

    return new FunctionalGyro(calibrator, angleSupplier, rateSupplier, rotationSupplier, resetter);
  }

  // TODO: Extract this into a separate class, similar to that for Pigeon2.
  // /** Helper function to wrap the XRPGyro type from WPILib. */
  // static IGyro wrapYawGyro(XRPGyro g) {
  // return new OffsetGyro(new FunctionalGyro(
  // ()
  // -> { System.out.println(">>> Null-op: XRPGyro doesn't calibrate."); },
  // ()
  // -> Degrees.of(g.getAngleZ()),
  // ()
  // -> DegreesPerSecond.of(g.getRateZ()),
  // ()
  // -> { return Rotation2d.fromDegrees(g.getAngleZ()); },
  // ()
  // -> {
  // // Note that this won't actually get invoked, because the OffsetGyro will
  // // instead just reset its offset value.
  // }));
  // }

  // TODO: Extract this into a separate class, similar to that for Pigeon2.
  // /** Helper function to wrap the RomiGyro type from WPILib. */
  // static IGyro wrapYawGyro(RomiGyro g) {
  // return new OffsetGyro(new FunctionalGyro(
  // ()
  // -> { System.out.println(">>> Null-op: RomiGyro doesn't calibrate."); },
  // ()
  // -> Degrees.of(g.getAngleZ()),
  // ()
  // -> DegreesPerSecond.of(g.getRateZ()),
  // ()
  // -> { return Rotation2d.fromDegrees(g.getAngleZ()); },
  // ()
  // -> {
  // // Note that this won't actually get invoked, because the OffsetGyro will
  // // instead just reset its offset value.
  // }));
  // }
}
