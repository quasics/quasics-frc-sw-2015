// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>

// clang-format off
/**
 * This defines a "wrapper" type that can be used so that any arbitrary type of
 * object representing an encoder may be used in a common way, even if they
 * don't share a common base class. (This wrapper provides a pretty basic view
 * of encoders, but that's the point; I'm just looking for a way to use all
 * kinds of an encoder as though there are a common/single kind of object.)
 *
 * As context:
 * <ul>
 *   <li>
 *     The WPILib framework provides a basic "Encoder" class, but the constructors
 *     (and implementation) assume that a specific type of electrical/signalling
 *     interface will be used to communicate with it.
 *   </li>
 *   <li>
 *     Other types of encoders (e.g., for the REV SparkMax controllers) provide
 *     similar functionality, but don't have a common base class (which would allow
 *     them to be used as a "drop-in" alternative via pointers or references), and
 *     also often don't even use the same function names or parameter/return types
 *     (which further restricts their drop-in use via other mechanisms, such as
 *     templates in C++ or Java "generics").
 *   </li>
 *   <li>
 *     However, they're *all* doing the same basic thing: keeping track of the data
 *     for a motor/wheel (distance, velocity/rate, acceleration, etc.).
 *   </li>
 *   <li>
 *     An additional concern is that the WPILib team is taking active steps to move
 *     some other kinds of classes *further* away from having a common type (see the
 *     comments on the IGyro class), which would suggest that this state of affairs
 *     is unlikely to change for the better anytime soon.
 *   </li>
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
// clang-format on
class TrivialEncoder {
 public:
  virtual ~TrivialEncoder() = default;

  /** Returns the distance recorded by the encoder (in meters). */
  virtual units::meter_t getPosition() = 0;

  /** Returns the current speed reported by the encoder (in meters/sec). */
  virtual units::meters_per_second_t getVelocity() = 0;

  /** Resets the encoder's distance. */
  virtual void reset() = 0;

  // Helper functions, making it easy to get IGyros.
 public:
  /** @return a stubbed version of a TrivialEncoder as a simple placeholder. */
  static inline TrivialEncoder& getNullEncoder();

  /**
   * Note: assumes that the underlying encoder has been configured to return
   * meter-based values for position/velocity.
   *
   * @return an TrivialEncoder wrapped around an <code>frc::Encoder</code>.
   */
  static std::unique_ptr<TrivialEncoder> wrapEncoder(frc::Encoder& encoder);

  /**
   * Note: assumes that the underlying encoder has been configured to return
   * meter-based values for position/velocity.
   *
   * @return an TrivialEncoder wrapped around an
   * <code>rev::RelativeEncoder</code>. */
  static std::unique_ptr<TrivialEncoder> wrapEncoder(
      rev::RelativeEncoder& encoder);
};

/**
 * Defines an TrivialEncoder subclass that uses std::function objects to
 * encapsulate the underyling behaviors.
 */
class FunctionalTrivialEncoder : public TrivialEncoder {
 public:
  using Runnable = std::function<void()>;
  using PositionSupplier = std::function<units::meter_t()>;
  using SpeedSupplier = std::function<units::meters_per_second_t()>;

 private:
  const PositionSupplier m_positionSupplier;
  const SpeedSupplier m_speedSupplier;
  const Runnable m_resetter;

 public:
  FunctionalTrivialEncoder(PositionSupplier positionSupplier,
                           SpeedSupplier rateSupplier, Runnable resetter)
      : m_positionSupplier(positionSupplier),
        m_speedSupplier(rateSupplier),
        m_resetter(resetter) {
  }
  void reset() override {
    m_resetter();
  }
  units::meter_t getPosition() override {
    return m_positionSupplier();
  }
  units::meters_per_second_t getVelocity() override {
    return m_speedSupplier();
  }
};

inline TrivialEncoder& TrivialEncoder::getNullEncoder() {
  static FunctionalTrivialEncoder nullEncoder{
      // getPosition
      [&]() { return units::meter_t(0); },
      // getVelocity
      [&]() { return units::meters_per_second_t(0); },
      // reset
      [&]() {}};
  return nullEncoder;
}
inline std::unique_ptr<TrivialEncoder> TrivialEncoder::wrapEncoder(
    frc::Encoder& encoder) {
  return std::unique_ptr<TrivialEncoder>(new FunctionalTrivialEncoder(
      [&]() { return units::meter_t(encoder.GetDistance()); },
      [&]() { return units::meters_per_second_t(encoder.GetRate()); },
      [&]() { encoder.Reset(); }));
};

inline std::unique_ptr<TrivialEncoder> TrivialEncoder::wrapEncoder(
    rev::RelativeEncoder& encoder) {
  return std::unique_ptr<TrivialEncoder>(new FunctionalTrivialEncoder(
      [&]() { return units::meter_t(encoder.GetPosition()); },
      [&]() { return units::meters_per_second_t(encoder.GetVelocity()); },
      [&]() { encoder.SetPosition(0); }));
};
