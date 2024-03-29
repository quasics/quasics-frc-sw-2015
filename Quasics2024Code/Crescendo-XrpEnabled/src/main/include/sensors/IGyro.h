// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "ConditionalCompileFlags.h"

#ifdef ENABLE_CTRE
#include <ctre/phoenix6/Pigeon2.hpp>
#endif

#ifdef ENABLE_XRP
#include <frc/xrp/XRPGyro.h>
#endif

#include <functional>

// clang-format off
/**
 * This defines a "wrapper" type that can be used to let any arbitrary "Gyro" or
 * ALU object be used in a common way, even if they don't share a common base
 * class.  (This wrapper provides a pretty basic view of gyros/ALUs, but that's
 * the point; I'm just looking for a way to use all kinds of an ALU as though
 * there are a common/single kind of object.)
 *
 * As context:
 * <ul>
 *   <li>
 *     Prior to the 2024 WPI tools, there was a common "Gyro" interface that
 *     many gyros/ALUs implemented. This allowed them to be used
 *     semi-interchangeably by code (e.g., if you're writing code that will
 *     run on robots that might not always have the same kind of ALU installed,
 *     such as a Pigeon2 over CAN on one drive base and an ADI ALU connected via
 *     SPI on another). This interface wasn't supported by all ALUs, but it was
 *     at least reasonably common.
 *   </li>
 *   <li>
 *     As a part of the updates for the 2024 WPI tools, this common interface
 *     was "deprecated" (meanining that it's going away in the future), and
 *     the WPI team removed its use immediately from the various classes for
 *     gyros/IMUs in the WPILib. (And other folks providing vendordeps for
 *     specific hardware are generally following suit.)
 *   </li>
 *   <li>
 *     These means gthat we are now effectively left without a common base type,
 *     which means that it's *much* harder to write one piece of code that will
 *     work with multiple ALUs, which is a real problem.
 *   </li>
 * </ul>
 *
 * So, I'm defining my own (minimal) "wrapper" interface, which can be used to
 * adapt any arbitrary gyro/ALU to a common type, along with some functions to
 * help encapsulate specific examples "real" gyro classes with the wrapper.
 *
 * @see https://refactoring.guru/design-patterns/decorator
 * @see https://en.wikipedia.org/wiki/Adapter_pattern
 */
// clang-format on
class IGyro {
  // Convenient type alises.
 public:
  using angle_t = units::degree_t;
  using rate_t = units::degrees_per_second_t;
#ifdef ENABLE_CTRE
  using Pigeon2 = ctre::phoenix6::hardware::Pigeon2;
#endif

  // Basic functions that any Gyro/ALU can generally support.
 public:
  /** Virtual destructor (needed in any base class). */
  virtual ~IGyro() = default;

  /**
   * Calibrates the gyro/ALU.  (Will be a "no-op" for some hardware that
   * auto-calibrates, such as the Pigeon2.)
   */
  virtual void calibrate() = 0;

  /** Returns the heading of the robot in degrees. */
  virtual angle_t getAngle() = 0;

  /** Returns the rate of rotation of the gyro. */
  virtual rate_t getRate() = 0;

  /** Returns the heading of the robot as a Rotation2d. */
  virtual frc::Rotation2d getRotation2d() = 0;

  /** Resets the gyro to 0. */
  virtual void reset() = 0;

  // Helper functions, making it easy to wrap specific gyros/ALUs inside of
  // IGyro objects.
 public:
  /** @return an IGyro wrapped around an <code>AnalogGyro</code>. */
  static inline std::unique_ptr<IGyro> wrapGyro(frc::AnalogGyro& g);

  /** @return an IGyro wrapped around an <code>ADXRS450_Gyro</code>. */
  static inline std::unique_ptr<IGyro> wrapGyro(frc::ADXRS450_Gyro& g);

#ifdef ENABLE_CTRE
  /**
   * @return an IGyro wrapped around the "yaw" axis for an <code>Pigeon2</code>.
   */
  static inline std::unique_ptr<IGyro> wrapYawGyro(Pigeon2& pigeon2);

  /**
   * @return an IGyro wrapped around the "yaw" axis for an <code>Pigeon2</code>.
   */
  static inline std::unique_ptr<IGyro> wrapGyro(Pigeon2& pigeon2) {
    return wrapYawGyro(pigeon2);
  }
#endif

#ifdef ENABLE_XRP
  /** @return an IGyro wrapped around an <code>XRPGyro</code>. */
  static inline std::unique_ptr<IGyro> wrapYawGyro(frc::XRPGyro& xrpGyro);
#endif

  // Other generators
 public:
  /**
   * @return a stubbed version of an IGyro as a simple placeholder.  This will
   * always return 0s for any value function, but at least gives you something
   * that can be "plugged into" code as a placeholder.
   */
  static inline IGyro& getNullGyro();
};

/**
 * Defines an IGyro subclass that uses std::function objects to encapsulate each
 * of the underlying behaviors.
 */
class FunctionalGyro : public IGyro {
  // Convenient type aliases.
 public:
  using Runnable = std::function<void()>;
  using AngleSupplier = std::function<angle_t()>;
  using RateSupplier = std::function<rate_t()>;
  using RotationSupplier = std::function<frc::Rotation2d()>;

 private:
  const Runnable m_calibrator;
  const AngleSupplier m_angleSupplier;
  const RateSupplier m_rateSupplier;
  const RotationSupplier m_rotationSupplier;
  const Runnable m_resetter;

 public:
  FunctionalGyro(Runnable calibrator, AngleSupplier angleSupplier,
                 RateSupplier rateSupplier, RotationSupplier rotationSupplier,
                 Runnable resetter)
      : m_calibrator(calibrator),
        m_angleSupplier(angleSupplier),
        m_rateSupplier(rateSupplier),
        m_rotationSupplier(rotationSupplier),
        m_resetter(resetter) {
  }

  void calibrate() override {
    m_calibrator();
  }

  angle_t getAngle() override {
    return m_angleSupplier();
  }

  rate_t getRate() override {
    return m_rateSupplier();
  }

  frc::Rotation2d getRotation2d() override {
    return m_rotationSupplier();
  }

  void reset() override {
    m_resetter();
  }
};

inline IGyro& IGyro::getNullGyro() {
  static FunctionalGyro nullGyro{// calibrate
                                 [] { return; },
                                 // getAngle
                                 [] { return angle_t(0); },
                                 // getRate
                                 [] { return rate_t(0); },
                                 // getRotation2d
                                 [] { return frc::Rotation2d{0_deg}; },
                                 // reset
                                 [] { return; }};
  return nullGyro;
}

inline std::unique_ptr<IGyro> IGyro::wrapGyro(frc::AnalogGyro& g) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() { g.Calibrate(); },
      // WPILib docs indicate that AnalogGyro::GetAngle() returns degrees
      [&]() { return units::degree_t(g.GetAngle()); },
      // WPILib docs indicate that AnalogGyro::GetRate() returns degrees/sec
      [&]() { return units::degrees_per_second_t(g.GetRate()); },
      [&]() { return g.GetRotation2d(); }, [&]() { g.Reset(); }));
}

inline std::unique_ptr<IGyro> IGyro::wrapGyro(frc::ADXRS450_Gyro& g) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() { g.Calibrate(); },
      // WPILib docs indicate that ADXRS450_Gyro::GetAngle() returns degrees
      [&]() { return units::degree_t(g.GetAngle()); },
      // WPILib docs indicate that ADXRS450_Gyro::GetRate() returns degrees/sec
      [&]() { return units::degrees_per_second_t(g.GetRate()); },
      [&]() { return g.GetRotation2d(); }, [&]() { g.Reset(); }));
}

#ifdef ENABLE_CTRE
std::unique_ptr<IGyro> IGyro::wrapYawGyro(IGyro::Pigeon2& pigeon2) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() {},
      // CTRE docs indicate that Pigeon2::GetAngle() returns degrees
      [&]() {
        return angle_t(pigeon2.GetAngle()); /* GetYaw().GetValue() */
      },
      // CTRE docs indicate that Pigeon2::GetRate() returns degrees/sec
      [&]() {
        return rate_t(pigeon2.GetRate()); /* GetAngularVelocityZ().GetValue() */
      },
      [&]() { return pigeon2.GetRotation2d(); },
      // Note that this will do a reset on the Pigeon for *all* axes.  A better
      // approach would be to further wrap this with an "OffsetGyro", similar to
      // what was used in the team's 2023 code, and in the "MattsToy" code for
      // 2024.
      [&]() { pigeon2.Reset(); }));
}
#endif

#ifdef ENABLE_XRP
std::unique_ptr<IGyro> IGyro::wrapYawGyro(frc::XRPGyro& xrpGyro) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() {},
      // CTRE docs indicate that XRPGyro::GetAngle() returns degrees
      [&]() {
        return angle_t(xrpGyro.GetAngle()); /* GetYaw().GetValue() */
      },
      // CTRE docs indicate that XRPGyro::GetRate() returns degrees/sec
      [&]() {
        return rate_t(xrpGyro.GetRate()); /* GetAngularVelocityZ().GetValue() */
      },
      [&]() { return units::degree_t{-xrpGyro.GetAngle()}; },
      // Note that this will do a reset on the XRPGyro for *all* axes.  A better
      // approach would be to further wrap this with an "OffsetGyro", similar to
      // what was used in the team's 2023 code, and in the "MattsToy" code for
      // 2024.
      [&]() { xrpGyro.Reset(); }));
}
#endif
