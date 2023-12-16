#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <functional>

/**
 * A convenient wrapper, representing basic functionality of any
 * gyro/single-axis ALU.
 *
 * This is required because WPILib is deprecating their definition of a common
 * <code>Gyro</code> base class, and thus there's no common base class that can
 * be used to manipulate different kinds of Gyros/ALUs in a consistent fashion.
 */
class IGyro {
  // Convenient type alises.
 public:
  using angle_t = units::degree_t;
  using rate_t = units::degrees_per_second_t;
  using Pigeon2 = ctre::phoenix6::hardware::Pigeon2;

 public:
  virtual ~IGyro() = default;

  /** Calibrates the gyro/ALU. */
  virtual void calibrate() = 0;

  /** Returns the heading of the robot in degrees. */
  virtual angle_t getAngle() = 0;

  /** Returns the rate of rotation of the gyro. */
  virtual rate_t getRate() = 0;

  /** Returns the heading of the robot as a Rotation2d. */
  virtual frc::Rotation2d getRotation2d() = 0;

  /** Resets the gyro to 0. */
  virtual void reset() = 0;

  // Helper functions, making it easy to get IGyros.
 public:
  /** @return a stubbed version of an IGyro as a simple placeholder. */
  static inline IGyro& getNullGyro();

  /** @return an IGyro wrapped around an <code>AnalogGyro</code>. */
  static inline std::unique_ptr<IGyro> wrapGyro(frc::AnalogGyro& g);

  /** @return an IGyro wrapped around an <code>ADXRS450_Gyro</code>. */
  static inline std::unique_ptr<IGyro> wrapGyro(frc::ADXRS450_Gyro& g);

  /** @return an IGyro wrapped around an <code>ADXRS450_Gyro</code>. */
  static inline std::unique_ptr<IGyro> wrapYawGyro(Pigeon2& pigeon2);
};

/**
 * Defines an IGyro subclass that uses std::function objects to encapsulate the
 * underyling behaviors.
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

std::unique_ptr<IGyro> IGyro::wrapYawGyro(IGyro::Pigeon2& pigeon2) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() {},
      // WPILib docs indicate that ADXRS450_Gyro::GetAngle() returns degrees
      [&]() {
        return angle_t(pigeon2.GetAngle()); /* GetYaw().GetValue() */
      },
      // WPILib docs indicate that ADXRS450_Gyro::GetRate() returns degrees/sec
      [&]() {
        return rate_t(pigeon2.GetRate()); /* GetAngularVelocityZ().GetValue() */
      },
      [&]() { return pigeon2.GetRotation2d(); },
      // Note that this will do a reset on the Pigeon for *all* axes.  A better
      // approach might be to use something like the "OffsetGyro" approach that
      // I prototyped in last year's "JavaUtilityLib", so that we can reset
      // just this *view* of the device.  (Though if someone resets the master
      // device, we'd still be stuck with a similar problem.)
      // TODO(mjh): Port something like the OffsetGyro into this year's
      // examples.
      [&]() { pigeon2.Reset(); }));
}