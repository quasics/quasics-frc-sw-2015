/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/AnalogInput.h>
#include <frc/DMA.h>
#include <frc/DMASample.h>
#include <frc/DigitalOutput.h>
#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {
  frc::DMA m_dma;  // DMA object

  // DMA needs a trigger, can use an output as trigger.
  // 8 Triggers exist per DMA object, can be triggered on any
  // DigitalSource.
  frc::DigitalOutput m_dmaTrigger{2};

  // Analog input to read with DMA
  frc::AnalogInput m_analogInput{0};

  // Encoder to read with DMA
  frc::Encoder m_encoder{0, 1};

 public:
  void RobotInit() override {
    // Trigger on falling edge of dma trigger output
    m_dma.SetExternalTrigger(&m_dmaTrigger, false, true);

    // Add inputs we want to read via DMA
    m_dma.AddAnalogInput(&m_analogInput);
    m_dma.AddEncoder(&m_encoder);
    m_dma.AddEncoderPeriod(&m_encoder);

    // Make sure trigger is set to off.
    m_dmaTrigger.Set(true);

    // Start DMA. No triggers or inputs can be added after this call
    // unless DMA is stopped.
    m_dma.StartDMA(1024);
  }

  void RobotPeriodic() override {
    // Manually Trigger DMA read
    m_dmaTrigger.Set(false);

    // Need to create a sample.
    frc::DMASample sample;
    int32_t remaining = 0;
    int32_t status = 0;
    // Update our sample. remaining is the number of samples remaining in the
    // buffer status is more specfic error messages if readStatus is not OK.
    // Wait 1ms if buffer is empty
    HAL_DMAReadStatus readStatus =
        sample.Update(&m_dma, 1_ms, &remaining, &status);

    if (readStatus == HAL_DMA_OK) {
      // Status value in all these reads should be checked, a non 0 value means
      // value could not be read

      // If DMA is good, values exist
      auto encoderDistance = sample.GetEncoderDistance(&m_encoder, &status);
      // Period is not scaled, and is a raw value
      auto encoderPeriod = sample.GetEncoderPeriodRaw(&m_encoder, &status);
      auto analogVoltage =
          sample.GetAnalogInputVoltage(&m_analogInput, &status);

      frc::SmartDashboard::PutNumber("Distance", encoderDistance);
      frc::SmartDashboard::PutNumber("Period", encoderPeriod);
      frc::SmartDashboard::PutNumber("Input", analogVoltage);
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
