#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

#include <functional>

class IGyro {
 public:
  using angle_t = double;  // units::degree_t
  using rate_t = double;   // units::degrees_per_second_t

 public:
  virtual ~IGyro() = default;

  virtual void calibrate() = 0;

  /** Returns the heading of the robot in degrees. */
  virtual angle_t getAngle() = 0;

  /** Returns the rate of rotation of the gyro. */
  virtual rate_t getRate() = 0;

  /** Returns the heading of the robot as a Rotation2d. */
  virtual frc::Rotation2d getRotation2d() = 0;

  virtual void reset() = 0;

  static inline IGyro& getNullGyro();

  static inline std::unique_ptr<IGyro> wrapGyro(frc::AnalogGyro& g);
  static inline std::unique_ptr<IGyro> wrapGyro(frc::ADXRS450_Gyro& g);
};

class FunctionalGyro : public IGyro {
 public:
  using Runnable = std::function<void()>;
  using AngleSupplier = std::function<angle_t()>;
  using RateSupplier = std::function<rate_t()>;  // units::degrees_per_second_t
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
      [&]() { g.Calibrate(); }, [&]() { return g.GetAngle(); },
      [&]() { return g.GetRate(); }, [&]() { return g.GetRotation2d(); },
      [&]() { g.Reset(); }));
}

inline std::unique_ptr<IGyro> IGyro::wrapGyro(frc::ADXRS450_Gyro& g) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() { g.Calibrate(); }, [&]() { return g.GetAngle(); },
      [&]() { return g.GetRate(); }, [&]() { return g.GetRotation2d(); },
      [&]() { g.Reset(); }));
}
