// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;

/**
 * This defines a "wrapper" type that can be used so that any arbitrary type of
 * object representing an encoder may be used in a common way, even if they
 * don't share a common base class. (This wrapper provides a pretty basic view
 * of encoders, but that's the point; I'm just looking for a way to use all
 * kinds of an encoder as though there are a common/single kind of object.)
 *
 * As context:
 * <ul>
 * <li>
 * The WPILib framework provides a basic "Encoder" class, but the constructors
 * (and implementation) assume that a specific type of electrical/signalling
 * interface will be used to communicate with it.
 * </li>
 * <li>
 * Other types of encoders (e.g., for the REV SparkMax controllers) provide
 * similar functionality, but don't have a common base class (which would allow
 * them to be used as a "drop-in" alternative via pointers or references), and
 * also often don't even use the same function names or parameter/return types
 * (which further restricts their drop-in use via other mechanisms, such as
 * templates in C++ or Java "generics").
 * </li>
 * <li>
 * However, they're *all* doing the same basic thing: keeping track of the data
 * for a motor/wheel (distance, velocity/rate, acceleration, etc.).
 * </li>
 * <li>
 * An additional concern is that the WPILib team is taking active steps to move
 * some other kinds of classes *further* away from having a common type (see the
 * comments on the IGyro class), which would suggest that this state of affairs
 * is unlikely to change for the better anytime soon.
 * </li>
 * </ul>
 *
 * So, I'm defining my own (minimal) "wrapper" interface, which can be used to
 * adapt any arbitrary encoder class/object to a common type, along with some
 * functions to help encapsulate specific examples "real" encoder classes with
 * the wrapper.
 * 
 * @see https://refactoring.guru/design-patterns/decorator
 * @see https://en.wikipedia.org/wiki/Adapter_pattern
 */
public interface TrivialEncoder {
  /** Returns the distance recorded by the encoder (in meters). */
  Measure<Distance> getPosition();

  /** Returns the current speed reported by the encoder (in meters/sec). */
  Measure<Velocity<Distance>> getVelocity();

  /** Resets the encoder's distance. */
  void reset();

  /** Creates a TrivialEncoder wrapper around a stock WPILib Encoder object. */
  public static TrivialEncoder forWpiLibEncoder(final Encoder encoder) {
    if (encoder == null) {
      throw new IllegalArgumentException("Null encoder");
    }

    return new TrivialEncoder() {
      @Override
      public Measure<Distance> getPosition() {
        return Meters.of(encoder.getDistance());
      }

      @Override
      public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond.of(encoder.getRate());
      }

      @Override
      public void reset() {
        encoder.reset();
      }
    };
  }
}
