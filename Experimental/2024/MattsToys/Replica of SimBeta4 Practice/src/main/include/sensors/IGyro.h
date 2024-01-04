#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/xrp/XRPGyro.h>
#include <units/angle.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <functional>

//Need to add these as libraries THIS!! FAILING BECAUSE 2023 ASK!!!
//CTR Phoenix -- https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json
//REV -- https://software-metadata.revrobotics.com/REVLib-2023.json
//XRP -- Where do u find this???


class IGyro {

 public:

 //for readibility

  using angle_t = units::degree_t;
  using rate_t = units::degrees_per_second_t;
  using Pigeon2 = ctre::phoenix6::hardware::Pigeon2;

 public:
 //will be overwritten. Creates the fundamental skeleton of the Gyro for interactive purposes
  virtual ~IGyro() = default;

  virtual void calibrate() = 0;

  virtual angle_t getAngle() = 0;


  virtual rate_t getRate() = 0;


  virtual frc::Rotation2d getRotation2d() = 0;


  virtual void reset() = 0;


 public:

  static inline IGyro& getNullGyro();

  //these are all the different gyro's that exsist and they all get grouped under this common interface
  
  static inline std::unique_ptr<IGyro> wrapGyro(frc::AnalogGyro& g);


  static inline std::unique_ptr<IGyro> wrapGyro(frc::ADXRS450_Gyro& g);

  static inline std::unique_ptr<IGyro> wrapYawGyro(Pigeon2& pigeon2);


  static inline std::unique_ptr<IGyro> wrapYawGyro(frc::XRPGyro& xrpGyro);
};


//this is like the RealDrivebase of the Idrivebase. Its inheriting. Here we define everything

class FunctionalGyro : public IGyro {

//for readibility

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
 //constructor

  FunctionalGyro(Runnable calibrator, AngleSupplier angleSupplier,
                 RateSupplier rateSupplier, RotationSupplier rotationSupplier,
                 Runnable resetter)
      : m_calibrator(calibrator),
        m_angleSupplier(angleSupplier),
        m_rateSupplier(rateSupplier),
        m_rotationSupplier(rotationSupplier),
        m_resetter(resetter) {
  }

//populating the skeletons creates in the parent class

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

//returns a completely empty and fake hyro. Im assuming for the minimzation of unexpected results
inline IGyro& IGyro::getNullGyro() {
  static FunctionalGyro nullGyro{
                                 [] { return; },
                                 [] { return angle_t(0); },
                                 [] { return rate_t(0); },
                                 [] { return frc::Rotation2d{0_deg}; },
                                 [] { return; }};
  return nullGyro;
}


//for all of these what its doing is taking the functions that are assigned to each different type of gyro
//and assiging it to the corresponding function. So later when this is wrapped. The program will know
//to just access this and automatically look up what function it shoud use

inline std::unique_ptr<IGyro> IGyro::wrapGyro(frc::AnalogGyro& g) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() { g.Calibrate(); },
      [&]() { return units::degree_t(g.GetAngle()); },
      [&]() { return units::degrees_per_second_t(g.GetRate()); },
      [&]() { return g.GetRotation2d(); }, [&]() { g.Reset(); }));
}

inline std::unique_ptr<IGyro> IGyro::wrapGyro(frc::ADXRS450_Gyro& g) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() { g.Calibrate(); },
      [&]() { return units::degree_t(g.GetAngle()); },
      [&]() { return units::degrees_per_second_t(g.GetRate()); },
      [&]() { return g.GetRotation2d(); }, [&]() { g.Reset(); }));
}

std::unique_ptr<IGyro> IGyro::wrapYawGyro(IGyro::Pigeon2& pigeon2) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() {},
      [&]() {
        return angle_t(pigeon2.GetAngle());
      },
      [&]() {
        return rate_t(pigeon2.GetRate());
      },
      [&]() { return pigeon2.GetRotation2d(); },
      [&]() { pigeon2.Reset(); }));
}

std::unique_ptr<IGyro> IGyro::wrapYawGyro(frc::XRPGyro& xrpGyro) {
  return std::unique_ptr<IGyro>(new FunctionalGyro(
      [&]() {},
      [&]() {
        return angle_t(xrpGyro.GetAngle());
      },
      [&]() {
        return rate_t(xrpGyro.GetRate()); 
      },
      [&]() { return units::degree_t{-xrpGyro.GetAngle()}; },
      [&]() { xrpGyro.Reset(); }));
}