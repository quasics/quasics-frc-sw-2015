#pragma once

#include <frc/Timer.h>

#include "subsystems/Lighting.h"

class LightingShifter {
 public:
  /**
   * @param lighting        lighting subsystem
   * @param cycleTime       period over which the lights should make a full
   *                        "sweep" through the strip
   * @param lightsMoveDown  if true, lights should appear to move from the upper
   *                        end of the strip (near the shooter) to the bottom;
   *                        otherwise, they will move from bottom to top
   */
  LightingShifter(Lighting* lighting, const units::second_t cycleTime,
                  bool lightsMoveDown)
      : m_lighting(lighting),
        m_timerInterval(cycleTime / lighting->GetNumberOfLEDs()),
        m_lightsMoveDown(lightsMoveDown),
        m_lastStartingPosition(0) {
  }

  void Start(Lighting::FrcColorFunction colorFunction) {
    m_timer.Reset();
    m_timer.Start();
    m_lastStartingPosition = 0;
    m_lighting->SetEachCellToColor(colorFunction);
  }

  void Update(Lighting::FrcColorFunction colorFunction) {
    if (IsTimeToUpdate()) {
      const auto stripSize = m_lighting->GetNumberOfLEDs();
      m_lastStartingPosition = (m_lastStartingPosition + 1) % stripSize;

      // Lighting::FrcColorFunction baseFunction = [this, stripSize](int pos) {
      //   const int slotsForColor = stripSize / 3;
      //   if (pos <= slotsForColor) {
      //     return frc::Color::kRed;
      //   } else if (pos <= (slotsForColor * 2)) {
      //     return frc::Color::kWhite;
      //   } else {
      //     return frc::Color::kBlue;
      //   }
      // };

      m_lighting->SetEachCellToColor([this, stripSize, colorFunction](int pos) {
        auto effectivePos =
            m_lightsMoveDown
                ? ((pos + m_lastStartingPosition + stripSize) % stripSize)
                : ((pos - m_lastStartingPosition + stripSize) % stripSize);
        return colorFunction(effectivePos);
      });
      m_timer.Reset();
    }
  }

  void Stop(frc::Color color = frc::Color::kBlack) {
    m_timer.Stop();
    m_lighting->SetAllToColor(color);
  }

 private:
  bool IsTimeToUpdate() {
    return m_timer.HasElapsed(m_timerInterval);
  }

 private:
  Lighting* m_lighting;
  const units::second_t m_timerInterval;
  const bool m_lightsMoveDown;
  int m_lastStartingPosition;
  frc::Timer m_timer;
};
