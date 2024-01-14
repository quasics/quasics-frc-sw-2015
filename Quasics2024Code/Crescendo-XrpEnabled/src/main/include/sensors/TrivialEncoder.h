#pragma once

#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>

// this is really similar to the IGyro. Basically there is no more common
// functionality so this wrapper is created

// Base Skeleton. Virtual = Overwritten;

class TrivialEncoder {
 public:
  virtual ~TrivialEncoder() = default;

  virtual units::meter_t getPosition() = 0;

  virtual units::meters_per_second_t getVelocity() = 0;

  virtual void reset() = 0;

 public:
  static inline TrivialEncoder& getNullEncoder();

  // the two unique types of encoders that will now be under 1 common system

  static std::unique_ptr<TrivialEncoder> wrapEncoder(frc::Encoder& encoder);

  static std::unique_ptr<TrivialEncoder> wrapEncoder(
      rev::RelativeEncoder& encoder);
};

// this should look awfully similar to the IGyro implementation.
//  just populating the skeleton.

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

// and now this is acocunting for the 2 different types of encoders that can be
// passed in and also for the instance that nothing gets passed in

inline TrivialEncoder& TrivialEncoder::getNullEncoder() {
  static FunctionalTrivialEncoder nullEncoder{
      [&]() { return units::meter_t(0); },
      [&]() { return units::meters_per_second_t(0); }, [&]() {}};
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