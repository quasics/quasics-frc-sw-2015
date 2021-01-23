/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Lights.h"
#include <frc2/command/SubsystemBase.h>

Lights::Lights() {
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

// This method will be called once per scheduler run
void Lights::Periodic() {}

void Lights::SetLightsColor(int red, int green, int blue){
    for (int i = 0; i<kLength; i++){
        m_ledBuffer[i].SetRGB(red, green, blue);
    }
    m_led.SetData(m_ledBuffer);
}

void Lights::TurnOff(){
    for (int i = 0; i<kLength; i++){
        m_ledBuffer[i].SetRGB(0, 0, 0);
    }
    m_led.SetData(m_ledBuffer);
}