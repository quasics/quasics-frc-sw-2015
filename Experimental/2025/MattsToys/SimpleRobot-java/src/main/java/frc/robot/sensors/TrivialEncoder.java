// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

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
 * @see <a href="https://refactoring.guru/design-patterns/decorator">Decorator
 *      pattern</a>
 * @see <a href="https://en.wikipedia.org/wiki/Adapter_pattern">Adapter
 *      pattern</a>
 */
public interface TrivialEncoder {
  /** Returns the distance recorded by the encoder (in meters). */
  Distance getPosition();

  /** Returns the current speed reported by the encoder (in meters/sec). */
  LinearVelocity getVelocity();

  /** Resets the encoder's distance. */
  void reset();

  /** Creates a TrivialEncoder wrapper around a stock WPILib Encoder object. */
  public static TrivialEncoder forWpiLibEncoder(final Encoder encoder) {
    if (encoder == null) {
      throw new IllegalArgumentException("Null encoder");
    }

    return new TrivialEncoder() {
      @Override
      public Distance getPosition() {
        return Meters.of(encoder.getDistance());
      }

      @Override
      public LinearVelocity getVelocity() {
        return MetersPerSecond.of(encoder.getRate());
      }

      @Override
      public void reset() {
        encoder.reset();
      }
    };
  }

  /** Creates a TrivialEncoder wrapper around a stock WPILib Encoder object. */
  public static TrivialEncoder forWpiLibEncoder(
      final Encoder encoder, final EncoderSim encoderSim) {
    if (encoder == null) {
      throw new IllegalArgumentException("Null Encoder");
    }
    if (encoderSim == null) {
      throw new IllegalArgumentException("Null EncoderSim");
    }

    return new TrivialEncoder() {
      @Override
      public Distance getPosition() {
        return Meters.of(encoder.getDistance());
      }

      @Override
      public LinearVelocity getVelocity() {
        return MetersPerSecond.of(encoder.getRate());
      }

      @Override
      public void reset() {
        encoder.reset();
        encoderSim.setCount(0);
      }
    };
  }

  public static final class NullEncoder implements TrivialEncoder {
    @Override
    public Distance getPosition() {
      return Meters.of(0);
    }

    @Override
    public LinearVelocity getVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void reset() {
    }
  }
}
