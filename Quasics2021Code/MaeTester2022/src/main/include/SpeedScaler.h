#pragma once

#include <algorithm>
#include <functional>

class SpeedScaler {
 public:
  enum Mode { Turtle, Normal, Turbo };
  SpeedScaler(
      std::function<Mode()> speedModeSupplier =
          [] { return SpeedScaler::Normal; },
      double normalMax = 0.6, double turtleMax = 0.4, double turboMax = 0.8)
      : m_speedModeSupplier(speedModeSupplier),
        m_turtleMax(std::clamp(turtleMax, 0.0, 1.0)),  // Bounds to [0..1]
        m_normalMax(std::clamp(normalMax, 0.0, 1.0)),  // Bounds to [0..1]
        m_turboMax(std::clamp(turboMax, 0.0, 1.0))     // Bounds to [0..1]
  {
  }

  double operator()(double input) const {
    double multiplier = m_normalMax;

    switch (m_speedModeSupplier()) {
      case Normal:
        multiplier = m_normalMax;
        break;
      case Turtle:
        multiplier = m_turtleMax;
        break;
      case Turbo:
        multiplier = m_turboMax;
        break;
    }
    return input * multiplier;
  }

 private:
  std::function<Mode()> m_speedModeSupplier;
  double m_turtleMax;
  double m_normalMax;
  double m_turboMax;
};

inline std::ostream& operator<<(std::ostream& os, SpeedScaler::Mode mode) {
  switch (mode) {
    case SpeedScaler::Normal:
      os << "Normal";
      break;
    case SpeedScaler::Turbo:
      os << "Turbo";
      break;
    case SpeedScaler::Turtle:
      os << "Turtle";
      break;
  }
  return os;
}