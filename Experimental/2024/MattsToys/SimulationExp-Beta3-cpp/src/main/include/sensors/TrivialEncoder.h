#pragma once

#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>

class TrivialEncoder {
 public:
  virtual ~TrivialEncoder() = default;

  /** Returns the distance recorded by the encoder (in meters). */
  virtual units::meter_t getPosition() = 0;

  /** Returns the current speed reported by the encoder (in meters/sec). */
  virtual units::meters_per_second_t getVelocity() = 0;

  /** Resets the encoder's distance. */
  virtual void reset() = 0;

  static inline TrivialEncoder& getNullEncoder();
};

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

  static std::unique_ptr<TrivialEncoder> forWpiLibEncoder(
      frc::Encoder& encoder) {
    return std::unique_ptr<TrivialEncoder>(new FunctionalTrivialEncoder(
        [&]() { return units::meter_t(encoder.GetDistance()); },
        [&]() { return units::meters_per_second_t(encoder.GetRate()); },
        [&]() { encoder.Reset(); }));
  };

  static std::unique_ptr<TrivialEncoder> forRevEncoder(
      rev::RelativeEncoder& encoder) {
    return std::unique_ptr<TrivialEncoder>(new FunctionalTrivialEncoder(
        [&]() { return units::meter_t(encoder.GetPosition()); },
        [&]() { return units::meters_per_second_t(encoder.GetVelocity()); },
        [&]() { encoder.SetPosition(0); }));
  };
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